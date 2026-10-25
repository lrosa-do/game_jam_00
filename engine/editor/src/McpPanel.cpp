#include "prisma3d/editor/McpPanel.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSettings.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/Theme.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kPadding = 6.0f;
const float kSpacing = 6.0f;
const float kPortWidth = 70.0f;
const float kTimeWidth = 70.0f;
const float kToolWidth = 150.0f;
const float kArgumentsWidth = 360.0f;
const float kResultWidth = 360.0f;

bool parsePort(const ct::String& text, int& port)
{
    const ct::String digits = text.trimmed();
    if (digits.empty() || digits.size() > 5) return false;
    int value = 0;
    for (std::size_t i = 0; i < digits.size(); ++i)
    {
        if (digits[i] < '0' || digits[i] > '9') return false;
        value = value * 10 + (digits[i] - '0');
    }
    if (value < 1024 || value > 65535) return false;
    port = value;
    return true;
}

ct::String portText(int port)
{
    ct::String text;
    text.append_number(port);
    return text;
}

} // namespace

McpPanel::McpPanel(McpServer& server, EditorSettings& settings)
        : mServer(server),
          mSettings(settings),
          mColumn(nullptr),
          mServerSwitch(nullptr),
          mPort(nullptr),
          mStatus(nullptr),
          mEdits(nullptr),
          mCode(nullptr),
          mCopy(nullptr),
          mClear(nullptr),
          mLog(nullptr),
          mShownRevision(0),
          mSyncing(false)
{
    mServer.setAllowEdits(mSettings.mcpAllowEdits);
    mServer.setAllowCode(mSettings.mcpAllowCode);
}

McpPanel::~McpPanel()
{
    mServer.stop();
}

bool McpPanel::attach(rt::Widget* parent)
{
    if (mColumn || !parent) return false;
    mColumn = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setStretch(1.0f);
    mColumn->setPadding(kPadding);
    mColumn->setSpacing(kSpacing);

    rt::BoxLayout* header = mColumn->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    header->setPadding(0.0f);
    header->setSpacing(kSpacing);
    mServerSwitch = header->createChild<rt::Switch>(ct::String(kServerText));
    mServerSwitch->setTooltip("Start or stop the local MCP server (127.0.0.1 only, no token). "
                              "It never starts by itself.");
    mServerSwitch->toggled.connect([this](bool on) { serverToggled(on); });
    rt::Label* portLabel = header->createChild<rt::Label>(ct::String("Port"));
    portLabel->setColor(rt::Theme::instance().textDisabled);
    mPort = header->createChild<CommitInput>();
    mPort->setSize(kPortWidth, 0.0f);
    mPort->setTooltip("TCP port on 127.0.0.1 (1024-65535). Stop the server to change it.");
    mPort->setCommittedText(portText(mSettings.mcpPort));
    mPort->setText(portText(mSettings.mcpPort));
    mPort->committed.connect([this](const ct::String& text) { portCommitted(text); });
    mStatus = header->createChild<rt::Label>(ct::String(kStoppedText));
    mStatus->setStretch(1.0f);
    mCopy = header->createChild<rt::Button>(ct::String("Copy command"));
    mCopy->setIconId(EditorIcons::get("Duplicate"));
    mCopy->setTooltip("Copy the 'claude mcp add' command that connects Claude Code to this "
                      "server");
    mCopy->clicked.connect([this] { copyCommand(); });
    mClear = header->createChild<rt::Button>(ct::String("Clear log"));
    mClear->setIconId(EditorIcons::get("Remove"));
    mClear->setTooltip("Clear the call log");
    mClear->clicked.connect([this] { mServer.clearLog(); });

    rt::BoxLayout* permissions = mColumn->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    permissions->setPadding(0.0f);
    permissions->setSpacing(kSpacing * 2.0f);
    mEdits = permissions->createChild<rt::Switch>(ct::String(kEditsText));
    mEdits->setTooltip("Let the AI change the scene, materials and project files. Every change "
                       "is an undo step you can reverse with Ctrl+Z.");
    mEdits->setOn(mSettings.mcpAllowEdits);
    mEdits->toggled.connect(
            [this](bool on)
            {
                if (mSyncing) return;
                mServer.setAllowEdits(on);
                mSettings.mcpAllowEdits = on;
                mSettings.flush();
            });
    mCode = permissions->createChild<rt::Switch>(ct::String(kCodeText));
    mCode->setTooltip("Let the AI write and attach scripts and press Play. Scripts run inside "
                      "the editor with your user's rights: turn this on only for requests you "
                      "trust.");
    mCode->setOn(mSettings.mcpAllowCode);
    mCode->toggled.connect(
            [this](bool on)
            {
                if (mSyncing) return;
                mServer.setAllowCode(on);
                mSettings.mcpAllowCode = on;
                mSettings.flush();
            });

    mLog = mColumn->createChild<rt::DataGrid>();
    mLog->setStretch(1.0f);
    mLog->setReadOnly(true);
    mLog->addColumn("Time", kTimeWidth, false);
    mLog->addColumn("Tool", kToolWidth, false);
    mLog->addColumn("Arguments", kArgumentsWidth, false);
    mLog->addColumn("Result", kResultWidth, false);

    syncStatus();
    syncLog();
    return true;
}

bool McpPanel::attached() const
{
    return mColumn != nullptr;
}

void McpPanel::update()
{
    mServer.poll();
    syncStatus();
    syncLog();
}

bool McpPanel::start()
{
    if (!mColumn) return false;
    int port = mSettings.mcpPort;
    if (mPort && !parsePort(mPort->text(), port))
    {
        mPort->setText(portText(mSettings.mcpPort));
        port = mSettings.mcpPort;
    }
    mSettings.mcpPort = port;
    const bool started = mServer.start(port);
    if (!started && mServerSwitch && mServerSwitch->isOn())
    {
        mSyncing = true;
        mServerSwitch->setOn(false);
        mSyncing = false;
    }
    if (started && mServerSwitch && !mServerSwitch->isOn())
    {
        mSyncing = true;
        mServerSwitch->setOn(true);
        mSyncing = false;
    }
    syncStatus();
    mSettings.flush();
    return started;
}

void McpPanel::stop()
{
    mServer.stop();
    if (mServerSwitch && mServerSwitch->isOn())
    {
        mSyncing = true;
        mServerSwitch->setOn(false);
        mSyncing = false;
    }
    syncStatus();
}

void McpPanel::serverToggled(bool on)
{
    if (mSyncing) return;
    if (on) start();
    else
        stop();
}

void McpPanel::portCommitted(const ct::String& text)
{
    int port = 0;
    if (mServer.running() || !parsePort(text, port))
    {
        mPort->setText(portText(mServer.running() ? mServer.port() : mSettings.mcpPort));
        mPort->setCommittedText(mPort->text());
        return;
    }
    mSettings.mcpPort = port;
    mSettings.flush();
}

bool McpPanel::copyCommand()
{
    Device* device = Device::singleton();
    if (!device) return false;
    const int port = mServer.running() ? mServer.port() : mSettings.mcpPort;
    device->clipboardSet(McpServer::addCommandFor(port).c_str());
    rt::Toast::show("Copied the 'claude mcp add' command", rt::Toast::Type::Success);
    return true;
}

ct::String McpPanel::statusText() const
{
    if (mServer.running())
        return ct::String("Listening on ") + McpServer::kAddress + ":" + portText(mServer.port()) +
               McpServer::kEndpoint;
    if (!mServer.error().empty()) return ct::String("Error: ") + mServer.error();
    return kStoppedText;
}

void McpPanel::syncStatus()
{
    if (!mStatus) return;
    const ct::String text = statusText();
    if (mStatus->text() != text) mStatus->setText(text);
    const rt::Theme& theme = rt::Theme::instance();
    if (mServer.running()) mStatus->setColor(rt::Color(110, 200, 130, 255));
    else if (!mServer.error().empty())
        mStatus->setColor(rt::Color(230, 100, 100, 255));
    else
        mStatus->setColor(theme.textDisabled);
    if (mPort) mPort->setEnabled(!mServer.running());
    if (mServerSwitch && mServerSwitch->isOn() != mServer.running())
    {
        mSyncing = true;
        mServerSwitch->setOn(mServer.running());
        mSyncing = false;
    }
}

void McpPanel::syncLog()
{
    if (!mLog || mShownRevision == mServer.logRevision()) return;
    mShownRevision = mServer.logRevision();
    mLog->clearRows();
    const ct::Vector<McpServer::LogEntry>& log = mServer.log();
    for (std::size_t i = log.size(); i > 0; --i)
    {
        const McpServer::LogEntry& entry = log[i - 1];
        ct::Vector<rt::String> cells;
        cells.push_back(entry.time);
        cells.push_back(entry.tool);
        cells.push_back(entry.arguments);
        cells.push_back(entry.ok ? ct::String("ok") : ct::String("error: ") + entry.outcome);
        mLog->addRow(cells);
    }
}

McpServer& McpPanel::server() const
{
    return mServer;
}

rt::Switch* McpPanel::serverSwitch() const
{
    return mServerSwitch;
}

CommitInput* McpPanel::portInput() const
{
    return mPort;
}

rt::Label* McpPanel::statusLabel() const
{
    return mStatus;
}

rt::Switch* McpPanel::editsSwitch() const
{
    return mEdits;
}

rt::Switch* McpPanel::codeSwitch() const
{
    return mCode;
}

rt::Button* McpPanel::copyButton() const
{
    return mCopy;
}

rt::Button* McpPanel::clearButton() const
{
    return mClear;
}

rt::DataGrid* McpPanel::logGrid() const
{
    return mLog;
}

} // namespace prisma3d
