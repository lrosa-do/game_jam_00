#include "scene.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace testbed
{

namespace
{
// Function-local so it exists before any static initialiser registers a scene.
ct::Vector<SceneEntry>& registry()
{
    static ct::Vector<SceneEntry> entries;
    return entries;
}
} // namespace

usize registerScene(const char* name, Category category, SceneFactory create)
{
    registry().push_back(SceneEntry{name, category, create});
    return registry().size() - 1;
}

usize sceneCount()
{
    return registry().size();
}

const SceneEntry& sceneAt(usize index)
{
    return registry()[index];
}

const char* sceneName(usize index)
{
    return index < registry().size() ? registry()[index].name : "?";
}

Scene* createScene(usize index)
{
    if (index >= registry().size())
        return nullptr;
    return registry()[index].create();
}

usize findScene(const char* text)
{
    for (usize i = 0; i < registry().size(); ++i)
        if (strcmp(registry()[i].name, text) == 0)
            return i;
    for (usize i = 0; i < registry().size(); ++i)
        if (strstr(registry()[i].name, text))
            return i;
    return registry().size();
}

const char* categoryName(Category category)
{
    switch (category)
    {
    case Category::Physics2D: return "2D physics";
    case Category::Physics3D: return "3D physics";
    case Category::AI: return "AI";
    }
    return "?";
}

void Scene::addCheck(const char* label)
{
    Check check;
    check.label = label;
    mChecks.push_back(check);
}

void Scene::report(usize index, bool ok, const char* detail)
{
    if (index >= mChecks.size())
        return;
    Check& check = mChecks[index];
    check.passed = ok;
    if (!ok)
        check.everFailed = true;
    check.detail = detail ? detail : "";
}

void Scene::resetChecks()
{
    for (Check& check : mChecks)
    {
        check.passed = true;
        check.everFailed = false;
        check.detail = ct::String();
    }
}

namespace
{
const char* const kKeyNames[usize(Key::Count)] = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S",
    "T", "U", "V", "W", "X", "Y", "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "Up",
    "Down", "Left", "Right", "PageUp", "PageDown", "Home", "End", "Insert", "Delete", "Tab",
    "Shift", "Ctrl", "Enter", "Backspace", "Minus", "Equals", "LeftBracket", "RightBracket",
    "Comma", "Period"};
} // namespace

const char* keyName(Key key)
{
    return usize(key) < usize(Key::Count) ? kKeyNames[usize(key)] : "?";
}

bool keyFromName(const char* name, usize length, Key& out)
{
    for (usize i = 0; i < usize(Key::Count); ++i)
    {
        const char* candidate = kKeyNames[i];
        if (strlen(candidate) != length)
            continue;
        bool same = true;
        for (usize c = 0; c < length && same; ++c)
        {
            char a = name[c];
            char b = candidate[c];
            if (a >= 'a' && a <= 'z')
                a = char(a - 'a' + 'A');
            if (b >= 'a' && b <= 'z')
                b = char(b - 'a' + 'A');
            same = a == b;
        }
        if (same)
        {
            out = Key(i);
            return true;
        }
    }
    return false;
}

float approach(float value, float target, float rate, float dt)
{
    const float step = rate * dt;
    if (value < target)
        return value + step < target ? value + step : target;
    return value - step > target ? value - step : target;
}

float rampControl(float value, float command, float centre, float rate, float returnRate, float dt)
{
    if (command == 0.0f)
        return approach(value, centre, returnRate, dt);
    const float target = centre + command * (command > 0.0f ? 1.0f - centre : 1.0f + centre);
    const bool reversing = (target - centre) * (value - centre) < 0.0f;
    return approach(value, target, reversing && returnRate > rate ? returnRate : rate, dt);
}

void addReadout(ct::Vector<Readout>& out, const char* label, const char* format, ...)
{
    Readout r;
    snprintf(r.label, sizeof(r.label), "%s", label);
    va_list args;
    va_start(args, format);
    vsnprintf(r.value, sizeof(r.value), format, args);
    va_end(args);
    out.push_back(r);
}

} // namespace testbed
