#include "prisma3d/editor/ConsolePanel.h"

#include "prisma3d/editor/EditorSettings.h"

#include "platform.h"

namespace prisma3d
{

namespace rt = ig::retained;

ConsolePanel::ConsolePanel()
        : mConsole(nullptr),
          mSettings(nullptr),
          mCursor(0),
          mReceived(0),
          mCapturing(false)
{
}

ConsolePanel::~ConsolePanel()
{
    captureLog(false);
}

rt::LogLevel ConsolePanel::levelOf(int platformLevel)
{
    switch (platformLevel)
    {
        case LOGLEVEL_DEBUG:
            return rt::LogLevel::Trace;
        case LOGLEVEL_WARN:
            return rt::LogLevel::Warn;
        case LOGLEVEL_ERROR:
        case LOGLEVEL_OFF:
            return rt::LogLevel::Error;
        default:
            return rt::LogLevel::Info;
    }
}

bool ConsolePanel::attach(rt::Widget* parent)
{
    if (mConsole || !parent) return false;
    mConsole = parent->createChild<rt::ConsoleWidget>();
    mConsole->setStretch(1.0f);
    mConsole->setMaxLines(static_cast<int>(EditorLog::kLimit));
    mConsole->onCommand.connect([this](const ct::String& text) { command(text); });
    update();
    return true;
}

void ConsolePanel::bindSettings(EditorSettings* settings)
{
    mSettings = settings;
    if (!mConsole || !mSettings) return;
    mConsole->setFilter(rt::LogLevel::Trace, mSettings->consoleTrace);
    mConsole->setFilter(rt::LogLevel::Info, mSettings->consoleInfo);
    mConsole->setFilter(rt::LogLevel::Warn, mSettings->consoleWarn);
    mConsole->setFilter(rt::LogLevel::Error, mSettings->consoleError);
    mConsole->setAutoScroll(mSettings->consoleAutoScroll);
}

bool ConsolePanel::attached() const
{
    return mConsole != nullptr;
}

void ConsolePanel::captureLog(bool capture)
{
    if (capture == mCapturing) return;
    mCapturing = capture;
    EditorLog::instance().capture(capture);
}

void ConsolePanel::update()
{
    if (!mConsole) return;
    if (mSettings)
    {
        mSettings->consoleTrace = mConsole->filter(rt::LogLevel::Trace);
        mSettings->consoleInfo = mConsole->filter(rt::LogLevel::Info);
        mSettings->consoleWarn = mConsole->filter(rt::LogLevel::Warn);
        mSettings->consoleError = mConsole->filter(rt::LogLevel::Error);
        mSettings->consoleAutoScroll = mConsole->autoScroll();
    }
    ct::Vector<EditorLog::Entry> entries;
    EditorLog::instance().read(mCursor, entries);
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        const EditorLog::Entry& entry = entries[i];
        const rt::LogLevel level = levelOf(entry.level);
        std::size_t start = 0;
        for (std::size_t k = 0; k <= entry.text.size(); ++k)
        {
            if (k != entry.text.size() && entry.text[k] != '\n') continue;
            if (k > start || entry.text.empty())
                mConsole->log(level, entry.text.substr(start, k - start));
            start = k + 1;
        }
        ++mReceived;
    }
}

rt::ConsoleWidget* ConsolePanel::console() const
{
    return mConsole;
}

std::size_t ConsolePanel::received() const
{
    return mReceived;
}

void ConsolePanel::command(const ct::String& text)
{
    if (!mConsole) return;
    mConsole->log(rt::LogLevel::Info, ct::String("> ") + text);
    mConsole->log(rt::LogLevel::Warn, "The editor console shows messages only; it has no "
                                      "command interpreter.");
}

} // namespace prisma3d
