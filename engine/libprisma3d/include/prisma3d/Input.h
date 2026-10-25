#pragma once

#include "mathc.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

enum class Key : int
{
    None = 0,
    Space = 32,
    Apostrophe = 39,
    Comma = 44,
    Minus = 45,
    Period = 46,
    Slash = 47,
    Zero = 48,
    One = 49,
    Two = 50,
    Three = 51,
    Four = 52,
    Five = 53,
    Six = 54,
    Seven = 55,
    Eight = 56,
    Nine = 57,
    Semicolon = 59,
    Equal = 61,
    A = 65,
    B = 66,
    C = 67,
    D = 68,
    E = 69,
    F = 70,
    G = 71,
    H = 72,
    I = 73,
    J = 74,
    K = 75,
    L = 76,
    M = 77,
    N = 78,
    O = 79,
    P = 80,
    Q = 81,
    R = 82,
    S = 83,
    T = 84,
    U = 85,
    V = 86,
    W = 87,
    X = 88,
    Y = 89,
    Z = 90,
    LeftBracket = 91,
    Backslash = 92,
    RightBracket = 93,
    Grave = 96,
    Escape = 256,
    Enter = 257,
    Tab = 258,
    Backspace = 259,
    Insert = 260,
    Delete = 261,
    Right = 262,
    Left = 263,
    Down = 264,
    Up = 265,
    PageUp = 266,
    PageDown = 267,
    Home = 268,
    End = 269,
    CapsLock = 280,
    NumLock = 281,
    PrintScreen = 282,
    Pause = 283,
    F1 = 290,
    F2 = 291,
    F3 = 292,
    F4 = 293,
    F5 = 294,
    F6 = 295,
    F7 = 296,
    F8 = 297,
    F9 = 298,
    F10 = 299,
    F11 = 300,
    F12 = 301,
    Kp0 = 320,
    Kp1 = 321,
    Kp2 = 322,
    Kp3 = 323,
    Kp4 = 324,
    Kp5 = 325,
    Kp6 = 326,
    Kp7 = 327,
    Kp8 = 328,
    Kp9 = 329,
    KpDecimal = 330,
    KpDivide = 331,
    KpMultiply = 332,
    KpSubtract = 333,
    KpAdd = 334,
    KpEnter = 335,
    KpEqual = 336,
    LeftShift = 340,
    LeftControl = 341,
    LeftAlt = 342,
    LeftSuper = 343,
    RightShift = 344,
    RightControl = 345,
    RightAlt = 346,
    RightSuper = 347,
    Menu = 348,
    ScrollLock = 349,
    Max = 350
};

enum KeyModifierMask : unsigned
{
    kModShift = 1u << 0,
    kModControl = 1u << 1,
    kModAlt = 1u << 2,
    kModSuper = 1u << 3,
    kModCapsLock = 1u << 4,
    kModNumLock = 1u << 5
};

enum class MouseButton : int
{
    Left = 0,
    Right = 1,
    Middle = 2,
    X1 = 3,
    X2 = 4,
    Max = 8
};

inline unsigned mouseButtonToMask(MouseButton button)
{
    return 1u << static_cast<int>(button);
}

enum class TouchPhase : int
{
    Down,
    Move,
    Up,
    Cancel
};

enum class InputEventType : int
{
    None,
    Key,
    Char,
    MouseMotion,
    MouseButton,
    MouseWheel,
    Touch,
    Action
};

struct InputEvent
{
    InputEventType type = InputEventType::None;
    union
    {
        struct
        {
            Key key;
            int scancode;
            bool pressed;
            bool echo;
            unsigned modifiers;
        } key;
        struct
        {
            std::uint32_t codepoint;
        } character;
        struct
        {
            int x, y;
            int relativeX, relativeY;
            unsigned buttonMask;
        } mouseMotion;
        struct
        {
            int x, y;
            MouseButton button;
            bool pressed;
            unsigned modifiers;
        } mouseButton;
        struct
        {
            int x, y;
            float deltaX, deltaY;
        } mouseWheel;
        struct
        {
            int id;
            float x, y, pressure;
            TouchPhase phase;
        } touch;
        struct
        {
            const char* name;
            bool pressed;
            float strength;
            int eventIndex;
        } action;
    };

    InputEvent()
            : type(InputEventType::None),
              key()
    {
    }
};

class InputMap;

class Input
{
public:
    typedef void (*EventDispatchFunction)(const InputEvent& event, void* user);

    static const int kMaxEvent = 32;

    Input();
    ~Input();
    Input(const Input&) = delete;
    Input& operator=(const Input&) = delete;

    static Input* singleton();
    InputMap& map();
    const InputMap& map() const;

    void beginFrame();
    std::uint64_t getProcessFrames() const;

    bool isAnythingPressed() const;
    bool isAnyKeyPressed() const;
    bool isKeyPressed(Key key) const;
    bool isMouseButtonPressed(MouseButton button) const;
    bool isKeyJustPressed(Key key) const;
    bool isKeyJustReleased(Key key) const;
    bool isMouseButtonJustPressed(MouseButton button) const;
    bool isMouseButtonJustReleased(MouseButton button) const;

    bool isActionPressed(const char* action, bool exact = false) const;
    bool isActionJustPressed(const char* action, bool exact = false) const;
    bool isActionJustReleased(const char* action, bool exact = false) const;
    float getActionStrength(const char* action, bool exact = false) const;
    float getActionRawStrength(const char* action, bool exact = false) const;
    float getAxis(const char* negativeAction, const char* positiveAction) const;
    Math::Vec2 getVector(const char* negativeX, const char* positiveX, const char* negativeY,
            const char* positiveY, float deadzone = -1.0f) const;
    void actionPress(const char* action, float strength = 1.0f);
    void actionRelease(const char* action);

    Math::Vec2 getMousePosition() const;
    Math::Vec2 getLastMouseVelocity();
    Math::Vec2 getMouseDelta() const;
    Math::Vec2 getMouseWheel() const;
    unsigned getMouseButtonMask() const;

    void warpMouse(const Math::Vec2& position);
    void setMousePosition(const Math::Vec2& position);

    void parseInputEvent(const InputEvent& event);
    void releasePressedEvents();
    void setApplicationFocused(bool focused);
    bool isApplicationFocused() const;

    void setEventDispatchFunction(EventDispatchFunction function, void* user);

    void setDisableInput(bool disable);
    bool isInputDisabled() const;
    void setPollingEnabled(bool enabled);
    bool isPollingEnabled() const;

private:
    struct VelocityTrack
    {
        std::uint64_t lastTick;
        Math::Vec2 velocity;
        Math::Vec2 accum;
        float accumT;
        float minRefFrame;
        float maxRefFrame;

        VelocityTrack();
        void update(const Math::Vec2& delta);
        void reset();
    };

    struct ActionState
    {
        std::uint64_t pressedProcessFrame;
        std::uint64_t releasedProcessFrame;
        bool exact;
        bool pressed[kMaxEvent];
        float strength[kMaxEvent];
        float rawStrength[kMaxEvent];
        bool apiPressed;
        float apiStrength;
        struct
        {
            bool pressed;
            float strength;
            float rawStrength;
        } cache;

        ActionState();
    };

    static Input* sSingleton;

    bool blocked() const;
    const ActionState* findAction(const char* action, bool exact) const;
    void updateActionCache(const char* action, ActionState& state);
    void parseActions(const InputEvent& event);

    InputMap* mMap;
    std::uint64_t mProcessFrames;
    bool mKeysPressed[static_cast<int>(Key::Max)];
    std::uint64_t mKeyPressedFrame[static_cast<int>(Key::Max)];
    std::uint64_t mKeyReleasedFrame[static_cast<int>(Key::Max)];
    std::uint64_t mMouseButtonPressedFrame[static_cast<int>(MouseButton::Max)];
    std::uint64_t mMouseButtonReleasedFrame[static_cast<int>(MouseButton::Max)];
    unsigned mMouseButtonMask;
    Math::Vec2 mMousePos;
    Math::Vec2 mMouseDelta;
    Math::Vec2 mMouseWheel;
    VelocityTrack mMouseVelocityTrack;
    ct::HashMap<ct::String, ActionState> mActionStates;
    bool mDisableInput;
    bool mPollingEnabled;
    bool mApplicationFocused;
    EventDispatchFunction mDispatch;
    void* mDispatchUser;
};

} // namespace prisma3d
