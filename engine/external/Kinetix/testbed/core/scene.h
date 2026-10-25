#pragma once

// scene.h - one demonstrable situation and the checks it evaluates every step.

#include "renderer.h"

#include <kinetix/core/Types.h>

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace testbed
{

using kinetix::u8;
using kinetix::usize;

// Keys a scene can read. Escape, Space (pause), P (manual/autopilot), F1 (help) and F2
// (chase camera) belong to the shell and never reach a scene.
enum class Key : u8
{
    A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Up, Down, Left, Right,
    PageUp, PageDown, Home, End, Insert, Delete,
    Tab, Shift, Ctrl, Enter, Backspace,
    Minus, Equals, LeftBracket, RightBracket, Comma, Period,
    Count
};

const char* keyName(Key key);
bool keyFromName(const char* name, usize length, Key& out);

// A game controller's axes, when one is plugged in: sticks -1..1 (y down
// positive, as the hardware reports it), triggers 0..1, dead zone removed.
enum class PadAxis : u8
{
    LeftX,
    LeftY,
    RightX,
    RightY,
    LeftTrigger,
    RightTrigger,
    Count
};

// Keyboard/controller state handed to a scene in manual mode before every physics step.
// `pressed` is set on the first step after the key went down.
struct ControlInput
{
    bool held[usize(Key::Count)] = {};
    bool pressed[usize(Key::Count)] = {};
    float pad[usize(PadAxis::Count)] = {};
    bool padConnected = false;

    bool down(Key key) const
    {
        return held[usize(key)];
    }
    bool hit(Key key) const
    {
        return pressed[usize(key)];
    }
    // -1, 0 or +1 from a pair of keys (either of two pairs).
    float pair(Key negative, Key positive) const
    {
        return (down(positive) ? 1.0f : 0.0f) - (down(negative) ? 1.0f : 0.0f);
    }
    float pair(Key negative, Key positive, Key negative2, Key positive2) const
    {
        const float v = (down(positive) || down(positive2) ? 1.0f : 0.0f) -
                        (down(negative) || down(negative2) ? 1.0f : 0.0f);
        return v;
    }
    float axis(PadAxis a) const
    {
        return padConnected ? pad[usize(a)] : 0.0f;
    }
};

struct Readout
{
    char label[40];
    char value[64];
};

// Moves `value` towards `target`, by at most `rate` per second.
float approach(float value, float target, float rate, float dt);
// A control surface on a key pair: `command` is -1, 0 or +1 (or a stick's value); held, the
// deflection runs towards it at `rate` per second, let go it returns to `centre` at `returnRate`.
float rampControl(float value, float command, float centre, float rate, float returnRate, float dt);
void addReadout(ct::Vector<Readout>& out, const char* label, const char* format, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 3, 4)))
#endif
    ;

struct Check
{
    ct::String label;
    bool passed = true;
    bool everFailed = false;
    ct::String detail;
};

class Scene
{
public:
    virtual ~Scene() = default;

    virtual const char* name() const = 0;
    virtual const char* description() const = 0;

    virtual void step(float dt) = 0;
    virtual void draw() = 0;
    // Re-evaluated after every step; a check that fails once stays flagged so
    // a transient failure cannot hide by recovering.
    virtual void evaluate() = 0;

    virtual bool is3D() const { return false; }

    struct Input
    {
        // 2D scenes: cursor in world units. 3D scenes: a ray from the eye through the cursor.
        kinetix::Math::vec3 worldPoint{0.0f};
        kinetix::Math::vec3 rayOrigin{0.0f};
        kinetix::Math::vec3 rayDirection{0.0f, 0.0f, -1.0f};

        bool grabPressed = false;
        bool grabHeld = false;
        bool grabReleased = false;
        bool firePressed = false;
        bool cutPressed = false;
    };

    // Called before step(), only when there is a window; a scene must work without it.
    virtual void handleInput(const Input& input) { (void)input; }

    // Manual mode: flown or driven from the keyboard. Headless runs are always scripted.
    // setManual(true) takes over the controls where the script left them; going back to the
    // script rebuilds the scene. Checks are not evaluated while manual.
    virtual bool drivable() const { return false; }
    virtual void setManual(bool manual) { mManual = manual; }
    bool manual() const { return mManual; }
    virtual void control(const ControlInput& input, float dt)
    {
        (void)input;
        (void)dt;
    }
    // One control per line, keys and description separated by a tab. Null: nothing to list.
    virtual const char* controlsHelp() const { return nullptr; }
    virtual void readouts(ct::Vector<Readout>& out) const { (void)out; }
    // Set by a scene that wants to start over: the shell rebuilds it in the same mode.
    bool restartRequested = false;
    // Display name of a scene to switch to; the shell clears it once acted on.
    const char* requestedScene = nullptr;

    // The shell shows one checkbox per name and sets the matching bit; what the bits mean is
    // the scene's business. Null means no overlays.
    virtual const char* const* overlayNames() const { return nullptr; }
    unsigned debugFlags = 0;
    virtual Camera2D camera2D() const { return Camera2D{}; }
    virtual Camera3D camera3D() const { return Camera3D{}; }

    const ct::Vector<Check>& checks() const { return mChecks; }
    bool allPassed() const
    {
        for (const Check& c : mChecks)
            if (c.everFailed)
                return false;
        return true;
    }

protected:
    void addCheck(const char* label);
    // Records the outcome for this step. `ok == false` sticks.
    void report(usize index, bool ok, const char* detail = nullptr);
    // Back to "not yet judged", for a scene that restarts itself.
    void resetChecks();

    ct::Vector<Check> mChecks;
    bool mManual = false;
};

// Scenes register themselves from their own file via REGISTER_SCENE, before main(), so the
// order is link order.
enum class Category : u8
{
    Physics2D,
    Physics3D,
    AI
};

using SceneFactory = Scene* (*)();

struct SceneEntry
{
    const char* name;
    Category category;
    SceneFactory create;
};

usize registerScene(const char* name, Category category, SceneFactory create);

usize sceneCount();
const SceneEntry& sceneAt(usize index);
const char* sceneName(usize index);
Scene* createScene(usize index);
// By exact display name, else the first whose name contains `text`; or the
// scene count when nothing matches.
usize findScene(const char* text);
const char* categoryName(Category category);

// The name is passed rather than read from a temporary: constructing a scene
// builds its whole world, and doing that at static-init time to ask its name
// would run physics before main().
#define REGISTER_SCENE(Type, displayName, category)                                                \
    namespace                                                                                      \
    {                                                                                              \
    Scene* create##Type() { return new Type(); }                                                   \
    const usize gRegistered##Type = registerScene(displayName, category, &create##Type);           \
    }

} // namespace testbed
