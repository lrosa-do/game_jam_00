#include "prisma3d/editor/EditorLog.h"

#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

#include <stdio.h>

namespace prisma3d
{

EditorLog::EditorLog()
        : mFirst(0),
          mCaptures(0)
{
}

EditorLog::~EditorLog()
{
}

EditorLog& EditorLog::instance()
{
    static EditorLog log;
    return log;
}

void EditorLog::sink(int level, const char* message, void* user)
{
    static const char* const kNames[] = { "DEBUG", "INFO", "WARN", "ERROR" };
    const char* name = level >= 0 && level < 4 ? kNames[level] : "LOG";
    fprintf(stderr, "[%s] %s\n", name, message);
    EditorLog* self = static_cast<EditorLog*>(user);
    if (self) self->post(level, message);
}

void EditorLog::print(const char* line, void* user)
{
    fprintf(stdout, "%s\n", line);
    EditorLog* self = static_cast<EditorLog*>(user);
    if (self) self->post(static_cast<int>(LOGLEVEL_INFO), line, true);
}

void EditorLog::capture(bool on)
{
    int captures;
    {
        ct::LockGuard guard(mLock);
        if (on) ++mCaptures;
        else if (mCaptures > 0)
            --mCaptures;
        captures = mCaptures;
    }
    if (on)
    {
        if (captures == 1)
            log_set_callback([](LogLevel level, const char* message, void* user)
                    { sink(static_cast<int>(level), message, user); }, this);
        if (ZenScripting* scripting = ZenScripting::instance())
            scripting->setPrintHandler(&EditorLog::print, this);
    }
    else if (!on && captures == 0)
    {
        log_set_callback(nullptr, nullptr);
        if (ZenScripting* scripting = ZenScripting::instance())
            scripting->setPrintHandler(nullptr, nullptr);
    }
}

bool EditorLog::capturing() const
{
    ct::LockGuard guard(mLock);
    return mCaptures > 0;
}

void EditorLog::post(int level, const char* text, bool script)
{
    ct::LockGuard guard(mLock);
    if (mEntries.size() >= kLimit)
    {
        const std::size_t drop = mEntries.size() - kLimit + 1;
        for (std::size_t i = 0; i + drop < mEntries.size(); ++i) mEntries[i] = mEntries[i + drop];
        for (std::size_t i = 0; i < drop; ++i) mEntries.pop_back();
        mFirst += drop;
    }
    Entry entry;
    entry.level = level;
    entry.script = script;
    entry.text = text ? text : "";
    mEntries.push_back(entry);
}

std::size_t EditorLog::total() const
{
    ct::LockGuard guard(mLock);
    return mFirst + mEntries.size();
}

std::size_t EditorLog::read(std::size_t& cursor, ct::Vector<Entry>& out) const
{
    ct::LockGuard guard(mLock);
    if (cursor < mFirst) cursor = mFirst;
    std::size_t count = 0;
    for (std::size_t i = cursor - mFirst; i < mEntries.size(); ++i)
    {
        out.push_back(mEntries[i]);
        ++count;
    }
    cursor = mFirst + mEntries.size();
    return count;
}

} // namespace prisma3d
