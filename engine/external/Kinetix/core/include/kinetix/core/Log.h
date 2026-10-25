#pragma once

#include <stdarg.h>

namespace kinetix
{

enum LogLevel
{
    LOG_INFO = 0,
    LOG_WARNING,
    LOG_ERROR,
    LOG_DEBUG
};

enum class LogMode
{
    None,
    Passive,
    Verbose
};

class Log
{
public:
    static void info(const char* fmt, ...);
    static void warning(const char* fmt, ...);
    static void error(const char* fmt, ...);
    static void debug(const char* fmt, ...);

    static void vwrite(LogLevel level, const char* fmt, va_list args);

    static void setMode(LogMode mode);
    static LogMode getMode();
    static bool accepts(LogLevel level);

    using Sink = void (*)(LogLevel level, const char* message);
    static void setSink(Sink sink);
};

} // namespace kinetix
