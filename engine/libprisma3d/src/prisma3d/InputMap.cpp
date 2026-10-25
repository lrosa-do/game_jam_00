#include "prisma3d/InputMap.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace
{

unsigned modifierMask(const InputEvent& event)
{
    switch (event.type)
    {
        case InputEventType::Key:
            return event.key.modifiers & (kModShift | kModControl | kModAlt | kModSuper);
        case InputEventType::MouseButton:
            return event.mouseButton.modifiers & (kModShift | kModControl | kModAlt | kModSuper);
        default:
            return 0;
    }
}

void reportStatus(bool isPressed, bool* pressed, float* strength, float* rawStrength)
{
    if (pressed) *pressed = isPressed;
    const float value = isPressed ? 1.0f : 0.0f;
    if (strength) *strength = value;
    if (rawStrength) *rawStrength = value;
}

} // namespace

InputMap* InputMap::sSingleton = nullptr;

InputMap::InputMap()
        : mLastId(1)
{
    sSingleton = this;
}

InputMap::~InputMap()
{
    if (sSingleton == this) sSingleton = nullptr;
}

InputMap* InputMap::singleton()
{
    return sSingleton;
}

bool InputMap::hasAction(const char* action) const
{
    return mMap.contains(ct::String(action));
}

void InputMap::addAction(const char* action, float deadzone)
{
    const ct::String key(action);
    if (mMap.contains(key))
    {
        log_error("input: action \"%s\" already exists", action);
        return;
    }
    Action& entry = mMap[key];
    entry.id = mLastId++;
    entry.deadzone = deadzone;
}

void InputMap::eraseAction(const char* action)
{
    if (!mMap.erase(ct::String(action))) log_error("input: unknown action \"%s\"", action);
}

float InputMap::actionGetDeadzone(const char* action) const
{
    const Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return 0.0f;
    }
    return entry->deadzone;
}

void InputMap::actionSetDeadzone(const char* action, float deadzone)
{
    Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    entry->deadzone = deadzone;
}

void InputMap::actionAddEvent(const char* action, const InputEvent& event)
{
    Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    if (findEvent(*entry, event, true, nullptr, nullptr, nullptr, nullptr)) return;
    if (entry->inputs.size() >= static_cast<size_t>(Input::kMaxEvent))
    {
        log_error("input: action \"%s\" cannot take more than %d events", action, Input::kMaxEvent);
        return;
    }
    entry->inputs.push_back(event);
}

bool InputMap::actionHasEvent(const char* action, const InputEvent& event) const
{
    const Action* entry = mMap.find(ct::String(action));
    if (!entry) return false;
    return findEvent(*entry, event, true, nullptr, nullptr, nullptr, nullptr) != nullptr;
}

void InputMap::actionEraseEvent(const char* action, const InputEvent& event)
{
    Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    for (size_t i = 0; i < entry->inputs.size(); ++i)
    {
        if (isMatch(entry->inputs[i], event, true))
        {
            entry->inputs.erase(entry->inputs.begin() + i);
            return;
        }
    }
}

void InputMap::actionEraseEvents(const char* action)
{
    Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    entry->inputs.clear();
}

const ct::Vector<InputEvent>* InputMap::actionGetEvents(const char* action) const
{
    const Action* entry = mMap.find(ct::String(action));
    return entry ? &entry->inputs : nullptr;
}

bool InputMap::eventIsAction(const InputEvent& event, const char* action, bool exactMatch) const
{
    return eventGetActionStatus(event, action, exactMatch, nullptr, nullptr, nullptr, nullptr);
}

int InputMap::eventGetIndex(const InputEvent& event, const char* action, bool exactMatch) const
{
    int index = -1;
    const bool matched =
            eventGetActionStatus(event, action, exactMatch, nullptr, nullptr, nullptr, &index);
    return matched ? index : -1;
}

bool InputMap::eventGetActionStatus(const InputEvent& event, const char* action, bool exactMatch,
        bool* pressed, float* strength, float* rawStrength, int* eventIndex) const
{
    const Action* entry = mMap.find(ct::String(action));
    if (!entry)
    {
        log_error("input: unknown action \"%s\"", action);
        return false;
    }

    if (event.type == InputEventType::Action)
    {
        const bool isPressed = event.action.pressed;
        if (pressed) *pressed = isPressed;
        const float value = isPressed ? event.action.strength : 0.0f;
        if (strength) *strength = value;
        if (rawStrength) *rawStrength = value;
        if (eventIndex)
        {
            *eventIndex = event.action.eventIndex >= 0 ? event.action.eventIndex
                                                       : static_cast<int>(entry->inputs.size());
        }
        return event.action.name && strcmp(event.action.name, action) == 0;
    }

    return findEvent(*entry, event, exactMatch, pressed, strength, rawStrength, eventIndex) !=
           nullptr;
}

const ct::HashMap<ct::String, InputMap::Action>& InputMap::getActionMap() const
{
    return mMap;
}

bool InputMap::actionMatch(const InputEvent& binding, const InputEvent& event, bool exactMatch,
        float, bool* pressed, float* strength, float* rawStrength)
{
    if (binding.type != event.type) return false;

    switch (binding.type)
    {
        case InputEventType::Key:
        {
            if (binding.key.key == Key::None) return false;
            bool match = binding.key.key == event.key.key;
            const unsigned actionMask = modifierMask(binding);
            const unsigned keyMask = modifierMask(event);
            if (event.key.pressed) match &= (actionMask & keyMask) == actionMask;
            if (exactMatch) match &= actionMask == keyMask;
            if (match) reportStatus(event.key.pressed, pressed, strength, rawStrength);
            return match;
        }
        case InputEventType::MouseButton:
        {
            bool match = binding.mouseButton.button == event.mouseButton.button;
            const unsigned actionMask = modifierMask(binding);
            const unsigned buttonMask = modifierMask(event);
            if (event.mouseButton.pressed) match &= (actionMask & buttonMask) == actionMask;
            if (exactMatch) match &= actionMask == buttonMask;
            if (match) reportStatus(event.mouseButton.pressed, pressed, strength, rawStrength);
            return match;
        }
        default:
            return false;
    }
}

bool InputMap::isMatch(const InputEvent& binding, const InputEvent& event, bool exactMatch)
{
    if (binding.type != event.type) return false;
    switch (binding.type)
    {
        case InputEventType::Key:
            return binding.key.key == event.key.key &&
                   (!exactMatch || modifierMask(binding) == modifierMask(event));
        case InputEventType::MouseButton:
            return binding.mouseButton.button == event.mouseButton.button &&
                   (!exactMatch || modifierMask(binding) == modifierMask(event));
        default:
            return false;
    }
}

const InputEvent* InputMap::findEvent(const Action& action, const InputEvent& event,
        bool exactMatch, bool* pressed, float* strength, float* rawStrength, int* eventIndex) const
{
    for (size_t i = 0; i < action.inputs.size(); ++i)
    {
        const InputEvent& binding = action.inputs[i];
        if (actionMatch(binding, event, exactMatch, action.deadzone, pressed, strength,
                    rawStrength))
        {
            if (eventIndex) *eventIndex = static_cast<int>(i);
            return &binding;
        }
    }
    return nullptr;
}

} // namespace prisma3d
