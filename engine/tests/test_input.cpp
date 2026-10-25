#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/InputMap.h"

#include "platform.h"

using namespace prisma3d;

namespace
{

int dispatched = 0;
InputEventType lastType = InputEventType::None;

void onEvent(const InputEvent& event, void* user)
{
    ++*static_cast<int*>(user);
    lastType = event.type;
}

InputEvent keyEvent(Key key, bool pressed, bool echo = false, unsigned modifiers = 0)
{
    InputEvent e;
    e.type = InputEventType::Key;
    e.key.key = key;
    e.key.scancode = 0;
    e.key.pressed = pressed;
    e.key.echo = echo;
    e.key.modifiers = modifiers;
    return e;
}

InputEvent buttonEvent(MouseButton button, bool pressed, int x, int y, unsigned modifiers = 0)
{
    InputEvent e;
    e.type = InputEventType::MouseButton;
    e.mouseButton.button = button;
    e.mouseButton.pressed = pressed;
    e.mouseButton.x = x;
    e.mouseButton.y = y;
    e.mouseButton.modifiers = modifiers;
    return e;
}

InputEvent actionEvent(const char* name, bool pressed, float strength = 1.0f)
{
    InputEvent e;
    e.type = InputEventType::Action;
    e.action.name = name;
    e.action.pressed = pressed;
    e.action.strength = strength;
    e.action.eventIndex = -1;
    return e;
}

bool near(float a, float b)
{
    const float d = a - b;
    return d < 0.001f && d > -0.001f;
}

InputEvent motionEvent(int x, int y, int dx, int dy)
{
    InputEvent e;
    e.type = InputEventType::MouseMotion;
    e.mouseMotion.x = x;
    e.mouseMotion.y = y;
    e.mouseMotion.relativeX = dx;
    e.mouseMotion.relativeY = dy;
    e.mouseMotion.buttonMask = 0;
    return e;
}

void test_singleton()
{
    CHECK(Input::singleton() == nullptr);
    CHECK(InputMap::singleton() == nullptr);
    {
        Input input;
        CHECK(Input::singleton() == &input);
        CHECK(InputMap::singleton() == &input.map());
    }
    CHECK(Input::singleton() == nullptr);
    CHECK(InputMap::singleton() == nullptr);
}

void test_keys()
{
    Input input;
    CHECK(!input.isAnythingPressed());
    CHECK(!input.isAnyKeyPressed());
    CHECK(!input.isKeyPressed(Key::Escape));

    input.parseInputEvent(keyEvent(Key::Escape, true));
    CHECK(input.isKeyPressed(Key::Escape));
    CHECK(input.isAnyKeyPressed());
    CHECK(input.isAnythingPressed());
    CHECK(!input.isKeyPressed(Key::Enter));

    input.parseInputEvent(keyEvent(Key::Escape, false, true));
    CHECK(input.isKeyPressed(Key::Escape));

    input.parseInputEvent(keyEvent(Key::Escape, false));
    CHECK(!input.isKeyPressed(Key::Escape));
    CHECK(!input.isAnyKeyPressed());

    input.parseInputEvent(keyEvent(Key::None, true));
    CHECK(!input.isAnyKeyPressed());
    input.parseInputEvent(keyEvent(Key::Max, true));
    CHECK(!input.isAnyKeyPressed());
    CHECK(!input.isKeyPressed(Key::Max));
    CHECK(!input.isKeyPressed(static_cast<Key>(-1)));
}

void test_mouse()
{
    Input input;
    CHECK(input.getMousePosition() == Math::Vec2(0.0f, 0.0f));
    CHECK(input.getMouseButtonMask() == 0);

    input.parseInputEvent(buttonEvent(MouseButton::Left, true, 10, 20));
    CHECK(input.isMouseButtonPressed(MouseButton::Left));
    CHECK(!input.isMouseButtonPressed(MouseButton::Right));
    CHECK(input.getMouseButtonMask() == mouseButtonToMask(MouseButton::Left));
    CHECK(input.isAnythingPressed());
    CHECK(input.getMousePosition() == Math::Vec2(10.0f, 20.0f));

    input.parseInputEvent(buttonEvent(MouseButton::Right, true, 10, 20));
    CHECK(input.getMouseButtonMask() ==
            (mouseButtonToMask(MouseButton::Left) | mouseButtonToMask(MouseButton::Right)));

    input.parseInputEvent(buttonEvent(MouseButton::Left, false, 11, 21));
    CHECK(!input.isMouseButtonPressed(MouseButton::Left));
    CHECK(input.isMouseButtonPressed(MouseButton::Right));
    CHECK(input.getMousePosition() == Math::Vec2(11.0f, 21.0f));

    input.parseInputEvent(motionEvent(30, 40, 19, 19));
    CHECK(input.getMousePosition() == Math::Vec2(30.0f, 40.0f));
}

void test_mouse_velocity()
{
    Input input;
    CHECK(input.getLastMouseVelocity() == Math::Vec2(0.0f, 0.0f));

    input.parseInputEvent(motionEvent(0, 0, 10, -5));
    time_sleep(60);
    input.parseInputEvent(motionEvent(10, -5, 10, -5));
    time_sleep(60);
    input.parseInputEvent(motionEvent(20, -10, 10, -5));
    const Math::Vec2 velocity = input.getLastMouseVelocity();
    CHECK(velocity.x > 0.0f);
    CHECK(velocity.y < 0.0f);
}

void test_map()
{
    Input input;
    InputMap& map = input.map();
    CHECK(!map.hasAction("jump"));
    map.addAction("jump");
    CHECK(map.hasAction("jump"));
    CHECK(near(map.actionGetDeadzone("jump"), InputMap::kDefaultDeadzone));
    map.actionSetDeadzone("jump", 0.5f);
    CHECK(near(map.actionGetDeadzone("jump"), 0.5f));
    map.addAction("jump");
    CHECK(map.getActionMap().size() == 1);

    const InputEvent space = keyEvent(Key::Space, true);
    const InputEvent left = buttonEvent(MouseButton::Left, true, 0, 0);
    CHECK(!map.actionHasEvent("jump", space));
    map.actionAddEvent("jump", space);
    map.actionAddEvent("jump", keyEvent(Key::Space, false));
    map.actionAddEvent("jump", left);
    CHECK(map.actionGetEvents("jump")->size() == 2);
    CHECK(map.actionHasEvent("jump", space));
    CHECK(map.actionHasEvent("jump", left));
    CHECK(!map.actionHasEvent("jump", keyEvent(Key::Enter, true)));
    CHECK(!map.actionHasEvent("jump", keyEvent(Key::Space, true, false, kModShift)));

    CHECK(map.eventIsAction(space, "jump"));
    CHECK(map.eventIsAction(keyEvent(Key::Space, false), "jump"));
    CHECK(map.eventIsAction(keyEvent(Key::Space, true, false, kModShift), "jump"));
    CHECK(!map.eventIsAction(keyEvent(Key::Space, true, false, kModShift), "jump", true));
    CHECK(!map.eventIsAction(keyEvent(Key::Enter, true), "jump"));
    CHECK(map.eventGetIndex(space, "jump") == 0);
    CHECK(map.eventGetIndex(left, "jump") == 1);
    CHECK(map.eventGetIndex(keyEvent(Key::Enter, true), "jump") == -1);
    CHECK(!map.eventIsAction(space, "missing"));
    CHECK(map.eventGetIndex(space, "missing") == -1);

    bool pressed = false;
    float strength = -1.0f;
    float raw = -1.0f;
    int index = -1;
    CHECK(map.eventGetActionStatus(keyEvent(Key::Space, false), "jump", false, &pressed, &strength,
            &raw, &index));
    CHECK(!pressed && strength == 0.0f && raw == 0.0f && index == 0);
    CHECK(map.eventGetActionStatus(actionEvent("jump", true, 0.5f), "jump", false, &pressed,
            &strength, &raw, &index));
    CHECK(pressed && near(strength, 0.5f) && near(raw, 0.5f) && index == 2);
    CHECK(!map.eventGetActionStatus(actionEvent("other", true), "jump", false, nullptr, nullptr,
            nullptr, nullptr));

    map.actionEraseEvent("jump", keyEvent(Key::Space, true));
    CHECK(map.actionGetEvents("jump")->size() == 1);
    CHECK(map.eventGetIndex(left, "jump") == 0);
    map.actionEraseEvents("jump");
    CHECK(map.actionGetEvents("jump")->size() == 0);
    map.eraseAction("jump");
    CHECK(!map.hasAction("jump"));
    CHECK(map.actionGetEvents("jump") == nullptr);
}

void test_actions()
{
    Input input;
    InputMap& map = input.map();
    map.addAction("jump");
    map.actionAddEvent("jump", keyEvent(Key::Space, true));
    map.actionAddEvent("jump", buttonEvent(MouseButton::Left, true, 0, 0));

    CHECK(!input.isActionPressed("jump"));
    CHECK(!input.isActionJustPressed("jump"));
    CHECK(!input.isActionPressed("missing"));
    CHECK(input.getActionStrength("jump") == 0.0f);

    input.beginFrame();
    input.parseInputEvent(keyEvent(Key::Space, true));
    CHECK(input.isActionPressed("jump"));
    CHECK(input.isActionJustPressed("jump"));
    CHECK(!input.isActionJustReleased("jump"));
    CHECK(input.getActionStrength("jump") == 1.0f);
    CHECK(input.getActionRawStrength("jump") == 1.0f);
    CHECK(input.isAnythingPressed());

    input.beginFrame();
    CHECK(input.isActionPressed("jump"));
    CHECK(!input.isActionJustPressed("jump"));

    input.parseInputEvent(buttonEvent(MouseButton::Left, true, 0, 0));
    CHECK(input.isActionPressed("jump"));
    CHECK(!input.isActionJustPressed("jump"));

    input.beginFrame();
    input.parseInputEvent(keyEvent(Key::Space, false));
    CHECK(input.isActionPressed("jump"));
    CHECK(!input.isActionJustReleased("jump"));

    input.parseInputEvent(buttonEvent(MouseButton::Left, false, 0, 0));
    CHECK(!input.isActionPressed("jump"));
    CHECK(input.isActionJustReleased("jump"));
    CHECK(input.getActionStrength("jump") == 0.0f);

    input.beginFrame();
    CHECK(!input.isActionJustReleased("jump"));

    input.parseInputEvent(keyEvent(Key::Space, true, false, kModShift));
    CHECK(input.isActionPressed("jump"));
    CHECK(!input.isActionPressed("jump", true));
    CHECK(input.isActionJustPressed("jump"));
    CHECK(!input.isActionJustPressed("jump", true));
    input.parseInputEvent(keyEvent(Key::Space, false, false, kModShift));
    CHECK(!input.isActionPressed("jump"));

    input.beginFrame();
    input.parseInputEvent(keyEvent(Key::Space, true, true));
    CHECK(input.isActionPressed("jump"));
    CHECK(input.isActionPressed("jump", true));
    input.parseInputEvent(keyEvent(Key::Space, false));

    input.setDisableInput(true);
    input.parseInputEvent(keyEvent(Key::Space, true));
    CHECK(!input.isActionPressed("jump"));
    input.setDisableInput(false);
    CHECK(input.isActionPressed("jump"));
    input.parseInputEvent(keyEvent(Key::Space, false));

    input.beginFrame();
    input.parseInputEvent(keyEvent(Key::Space, true));
    input.setApplicationFocused(false);
    CHECK(!input.isActionPressed("jump"));
    CHECK(input.isActionJustReleased("jump"));
    input.setApplicationFocused(true);
}

void test_virtual_actions()
{
    Input input;
    InputMap& map = input.map();
    map.addAction("fire");
    map.actionAddEvent("fire", keyEvent(Key::F, true));

    input.beginFrame();
    input.actionPress("fire", 0.5f);
    CHECK(input.isActionPressed("fire"));
    CHECK(input.isActionJustPressed("fire"));
    CHECK(near(input.getActionStrength("fire"), 0.5f));
    CHECK(near(input.getActionRawStrength("fire"), 0.5f));

    input.beginFrame();
    input.actionPress("fire", 2.0f);
    CHECK(!input.isActionJustPressed("fire"));
    CHECK(input.getActionStrength("fire") == 1.0f);

    input.parseInputEvent(keyEvent(Key::F, true));
    input.parseInputEvent(keyEvent(Key::F, false));
    CHECK(!input.isActionPressed("fire"));
    CHECK(input.isActionJustReleased("fire"));

    input.beginFrame();
    input.actionPress("fire");
    input.actionRelease("fire");
    CHECK(!input.isActionPressed("fire"));
    CHECK(input.isActionJustReleased("fire"));
    input.actionPress("missing");
    input.actionRelease("missing");
    CHECK(!input.isActionPressed("missing"));

    input.beginFrame();
    input.parseInputEvent(actionEvent("fire", true, 0.25f));
    CHECK(input.isActionPressed("fire"));
    CHECK(input.isActionJustPressed("fire"));
    CHECK(near(input.getActionStrength("fire"), 0.25f));
    input.parseInputEvent(actionEvent("fire", false));
    CHECK(!input.isActionPressed("fire"));
}

void test_axis_and_vector()
{
    Input input;
    InputMap& map = input.map();
    map.addAction("left");
    map.addAction("right");
    map.addAction("up");
    map.addAction("down");
    map.actionAddEvent("left", keyEvent(Key::A, true));
    map.actionAddEvent("right", keyEvent(Key::D, true));
    map.actionAddEvent("up", keyEvent(Key::W, true));
    map.actionAddEvent("down", keyEvent(Key::S, true));

    CHECK(input.getAxis("left", "right") == 0.0f);
    CHECK(input.getVector("left", "right", "up", "down") == Math::Vec2(0.0f, 0.0f));

    input.parseInputEvent(keyEvent(Key::D, true));
    CHECK(input.getAxis("left", "right") == 1.0f);
    CHECK(input.getVector("left", "right", "up", "down") == Math::Vec2(1.0f, 0.0f));

    input.parseInputEvent(keyEvent(Key::A, true));
    CHECK(input.getAxis("left", "right") == 0.0f);

    input.parseInputEvent(keyEvent(Key::A, false));
    input.parseInputEvent(keyEvent(Key::S, true));
    const Math::Vec2 diagonal = input.getVector("left", "right", "up", "down");
    CHECK(near(diagonal.Length(), 1.0f));
    CHECK(near(diagonal.x, 0.70710678f) && near(diagonal.y, 0.70710678f));

    input.actionRelease("right");
    input.actionRelease("down");
    input.actionPress("right", 0.5f);
    const Math::Vec2 half = input.getVector("left", "right", "up", "down");
    CHECK(near(half.x, (0.5f - 0.2f) / 0.8f) && half.y == 0.0f);
    CHECK(input.getVector("left", "right", "up", "down", 0.6f) == Math::Vec2(0.0f, 0.0f));
    CHECK(near(input.getVector("left", "right", "up", "down", 0.0f).x, 0.5f));
}

void test_focus_release()
{
    Input input;
    input.parseInputEvent(keyEvent(Key::W, true));
    input.parseInputEvent(buttonEvent(MouseButton::Middle, true, 0, 0));
    CHECK(input.isApplicationFocused());

    input.releasePressedEvents();
    CHECK(input.isKeyPressed(Key::W));
    CHECK(input.isMouseButtonPressed(MouseButton::Middle));

    input.setApplicationFocused(false);
    CHECK(!input.isApplicationFocused());
    CHECK(!input.isKeyPressed(Key::W));
    CHECK(!input.isMouseButtonPressed(MouseButton::Middle));
    CHECK(!input.isAnythingPressed());

    input.setApplicationFocused(true);
    input.parseInputEvent(keyEvent(Key::W, true));
    CHECK(input.isKeyPressed(Key::W));
}

void test_disable()
{
    Input input;
    input.parseInputEvent(keyEvent(Key::A, true));
    input.parseInputEvent(buttonEvent(MouseButton::Left, true, 0, 0));
    input.setDisableInput(true);
    CHECK(input.isInputDisabled());
    CHECK(!input.isKeyPressed(Key::A));
    CHECK(!input.isMouseButtonPressed(MouseButton::Left));
    CHECK(input.getMouseButtonMask() == 0);
    CHECK(!input.isAnythingPressed());
    input.setDisableInput(false);
    CHECK(input.isKeyPressed(Key::A));
    CHECK(input.isMouseButtonPressed(MouseButton::Left));
}

void test_dispatch()
{
    Input input;
    dispatched = 0;
    lastType = InputEventType::None;
    input.setEventDispatchFunction(onEvent, &dispatched);

    input.parseInputEvent(keyEvent(Key::B, true));
    CHECK(dispatched == 1);
    CHECK(lastType == InputEventType::Key);

    InputEvent wheel;
    wheel.type = InputEventType::MouseWheel;
    wheel.mouseWheel.x = 0;
    wheel.mouseWheel.y = 0;
    wheel.mouseWheel.deltaX = 0.0f;
    wheel.mouseWheel.deltaY = 1.0f;
    input.parseInputEvent(wheel);
    CHECK(dispatched == 2);
    CHECK(lastType == InputEventType::MouseWheel);

    input.setDisableInput(true);
    input.parseInputEvent(keyEvent(Key::B, false));
    CHECK(dispatched == 2);
    input.setDisableInput(false);

    input.setEventDispatchFunction(nullptr, nullptr);
    input.parseInputEvent(keyEvent(Key::B, true));
    CHECK(dispatched == 2);
}

} // namespace

void test_mouse_motion_tracker()
{
    MouseMotionTracker tracker;
    int rx = 99;
    int ry = 99;
    tracker.motion(100, 50, 0, 0, false, rx, ry);
    CHECK(rx == 0 && ry == 0);
    tracker.motion(110, 47, 0, 0, false, rx, ry);
    CHECK(rx == 10 && ry == -3);
    tracker.motion(110, 47, 0, 0, false, rx, ry);
    CHECK(rx == 0 && ry == 0);
    tracker.motion(105, 60, 0, 0, false, rx, ry);
    CHECK(rx == -5 && ry == 13);

    tracker.motion(105, 60, 7, -2, true, rx, ry);
    CHECK(rx == 7 && ry == -2);
    tracker.motion(120, 60, 0, 0, true, rx, ry);
    CHECK(rx == 15 && ry == 0);
    tracker.motion(121, 61, 4, 4, false, rx, ry);
    CHECK(rx == 1 && ry == 1);

    MouseMotionTracker raw;
    raw.motion(30, 30, 5, 6, true, rx, ry);
    CHECK(rx == 5 && ry == 6);
}

int main()
{
    test_singleton();
    test_keys();
    test_mouse();
    test_mouse_velocity();
    test_map();
    test_actions();
    test_virtual_actions();
    test_axis_and_vector();
    test_focus_release();
    test_disable();
    test_dispatch();
    test_mouse_motion_tracker();
    if (failures) printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
