#pragma once

#include "prisma3d/Input.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class InputMap
{
public:
    static constexpr float kDefaultDeadzone = 0.2f;

    struct Action
    {
        int id = 0;
        float deadzone = kDefaultDeadzone;
        ct::Vector<InputEvent> inputs;
    };

    InputMap();
    ~InputMap();
    InputMap(const InputMap&) = delete;
    InputMap& operator=(const InputMap&) = delete;

    static InputMap* singleton();

    bool hasAction(const char* action) const;
    void addAction(const char* action, float deadzone = kDefaultDeadzone);
    void eraseAction(const char* action);

    float actionGetDeadzone(const char* action) const;
    void actionSetDeadzone(const char* action, float deadzone);
    void actionAddEvent(const char* action, const InputEvent& event);
    bool actionHasEvent(const char* action, const InputEvent& event) const;
    void actionEraseEvent(const char* action, const InputEvent& event);
    void actionEraseEvents(const char* action);

    const ct::Vector<InputEvent>* actionGetEvents(const char* action) const;
    bool eventIsAction(const InputEvent& event, const char* action, bool exactMatch = false) const;
    int eventGetIndex(const InputEvent& event, const char* action, bool exactMatch = false) const;
    bool eventGetActionStatus(const InputEvent& event, const char* action, bool exactMatch,
            bool* pressed, float* strength, float* rawStrength, int* eventIndex) const;

    const ct::HashMap<ct::String, Action>& getActionMap() const;

    static bool actionMatch(const InputEvent& binding, const InputEvent& event, bool exactMatch,
            float deadzone, bool* pressed, float* strength, float* rawStrength);
    static bool isMatch(const InputEvent& binding, const InputEvent& event, bool exactMatch);

private:
    static InputMap* sSingleton;

    const InputEvent* findEvent(const Action& action, const InputEvent& event, bool exactMatch,
            bool* pressed, float* strength, float* rawStrength, int* eventIndex) const;

    ct::HashMap<ct::String, Action> mMap;
    int mLastId;
};

} // namespace prisma3d
