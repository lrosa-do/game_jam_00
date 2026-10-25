#pragma once

#include "prisma3d/editor/EditorLog.h"

#include <igui/widgets/ConsoleWidget.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/string.hpp>

#include <cstddef>

namespace prisma3d
{

class EditorSettings;

class ConsolePanel
{
public:
    ConsolePanel();
    ~ConsolePanel();
    ConsolePanel(const ConsolePanel&) = delete;
    ConsolePanel& operator=(const ConsolePanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void captureLog(bool capture);
    void bindSettings(EditorSettings* settings);
    void update();

    ig::retained::ConsoleWidget* console() const;
    std::size_t received() const;

    static ig::retained::LogLevel levelOf(int platformLevel);

private:
    void command(const ct::String& text);

    ig::retained::ConsoleWidget* mConsole;
    EditorSettings* mSettings;
    std::size_t mCursor;
    std::size_t mReceived;
    bool mCapturing;
};

} // namespace prisma3d
