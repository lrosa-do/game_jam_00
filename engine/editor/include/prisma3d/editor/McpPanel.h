#pragma once

#include "prisma3d/editor/McpServer.h"
#include "prisma3d/editor/PropertyWidgets.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/DataWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

class EditorSettings;

class McpPanel
{
public:
    static constexpr const char* kServerText = "MCP server";
    static constexpr const char* kEditsText = "Allow scene edits";
    static constexpr const char* kCodeText = "Allow scripts & Play";
    static constexpr const char* kStoppedText = "Stopped";

    McpPanel(McpServer& server, EditorSettings& settings);
    ~McpPanel();
    McpPanel(const McpPanel&) = delete;
    McpPanel& operator=(const McpPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void update();

    bool start();
    void stop();
    bool copyCommand();
    ct::String statusText() const;

    McpServer& server() const;
    ig::retained::Switch* serverSwitch() const;
    CommitInput* portInput() const;
    ig::retained::Label* statusLabel() const;
    ig::retained::Switch* editsSwitch() const;
    ig::retained::Switch* codeSwitch() const;
    ig::retained::Button* copyButton() const;
    ig::retained::Button* clearButton() const;
    ig::retained::DataGrid* logGrid() const;

private:
    void portCommitted(const ct::String& text);
    void serverToggled(bool on);
    void syncStatus();
    void syncLog();

    McpServer& mServer;
    EditorSettings& mSettings;
    ig::retained::BoxLayout* mColumn;
    ig::retained::Switch* mServerSwitch;
    CommitInput* mPort;
    ig::retained::Label* mStatus;
    ig::retained::Switch* mEdits;
    ig::retained::Switch* mCode;
    ig::retained::Button* mCopy;
    ig::retained::Button* mClear;
    ig::retained::DataGrid* mLog;
    unsigned int mShownRevision;
    bool mSyncing;
};

} // namespace prisma3d
