#pragma once

#include <ct/function.hpp>
#include <ct/http_server.hpp>
#include <ct/json.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

enum class McpPermission : int
{
    Read,
    Edit,
    Code
};

struct McpToolResult
{
    ct::Json content = ct::Json::array();
    bool isError = false;

    void addText(const ct::String& text);
    void addImage(const ct::String& base64, const char* mimeType);
    void fail(const ct::String& message);
};

class McpServer
{
public:
    static constexpr int kDefaultPort = 8787;
    static constexpr const char* kAddress = "127.0.0.1";
    static constexpr const char* kEndpoint = "/mcp";
    static constexpr const char* kServerName = "prisma3d-editor";
    static constexpr const char* kModernVersion = "2026-07-28";
    static constexpr const char* kLegacyVersions[] = { "2025-11-25", "2025-06-18", "2025-03-26",
        "2024-11-05" };
    static constexpr std::size_t kLegacyVersionCount = 4;
    static constexpr const char* kFallbackVersion = "2025-03-26";
    static constexpr std::size_t kLogLimit = 200;
    static constexpr std::size_t kSessionLimit = 16;
    static constexpr std::size_t kArgumentsLimit = 90;

    static constexpr int kParseError = -32700;
    static constexpr int kInvalidRequest = -32600;
    static constexpr int kMethodNotFound = -32601;
    static constexpr int kInvalidParams = -32602;
    static constexpr int kHeaderMismatch = -32020;
    static constexpr int kUnsupportedVersion = -32022;

    using ToolHandler = ct::Function<void(const ct::Json&, McpToolResult&)>;

    struct Tool
    {
        ct::String name;
        ct::String description;
        ct::Json schema;
        McpPermission level = McpPermission::Read;
        ToolHandler handler;
    };

    struct LogEntry
    {
        ct::String time;
        ct::String tool;
        ct::String arguments;
        ct::String outcome;
        bool ok = true;
    };

    McpServer();
    ~McpServer();
    McpServer(const McpServer&) = delete;
    McpServer& operator=(const McpServer&) = delete;

    void addTool(const char* name, const char* description, const ct::Json& schema,
            McpPermission level, const ToolHandler& handler);
    std::size_t toolCount() const;
    const Tool& tool(std::size_t index) const;
    const Tool* findTool(const ct::String& name) const;
    std::size_t listedToolCount() const;

    bool start(int port);
    void stop();
    bool running() const;
    int port() const;
    const ct::String& error() const;
    void poll();

    ct::String endpoint() const;
    ct::String addCommand() const;
    static ct::String endpointFor(int port);
    static ct::String addCommandFor(int port);

    void setAllowEdits(bool allow);
    bool allowEdits() const;
    void setAllowCode(bool allow);
    bool allowCode() const;
    bool permitted(McpPermission level) const;
    static const char* permissionSwitch(McpPermission level);

    const ct::Vector<LogEntry>& log() const;
    unsigned int logRevision() const;
    void clearLog();
    std::size_t sessionCount() const;

    static bool originAllowed(const ct::HttpRequest& request);
    static bool hostAllowed(const ct::HttpRequest& request);

private:
    struct Context
    {
        bool modern = false;
        ct::String version;
    };

    void handle(const ct::HttpRequest& request, ct::HttpResponse& response);
    void dispatch(const ct::Json& message, const ct::HttpRequest& request,
            ct::HttpResponse& response);
    bool validateModern(const ct::Json& message, const ct::HttpRequest& request,
            ct::HttpResponse& response);
    bool validateLegacy(const ct::Json& message, const ct::HttpRequest& request,
            ct::HttpResponse& response);
    void initialize(const ct::Json& message, ct::HttpResponse& response);
    void discover(const ct::Json& message, ct::HttpResponse& response);
    void listTools(const ct::Json& message, const Context& context, ct::HttpResponse& response);
    void callTool(const ct::Json& message, const Context& context, ct::HttpResponse& response);

    void reply(ct::HttpResponse& response, const ct::Json& id, ct::Json result,
            const Context& context);
    void replyError(ct::HttpResponse& response, int status, const ct::Json* id, int code,
            const ct::String& message, const ct::Json* data = nullptr);
    void record(const ct::String& tool, const ct::String& arguments, bool ok,
            const ct::String& outcome);
    static bool cleanArguments(const Tool& tool, const ct::Json& arguments, ct::Json& cleaned,
            ct::String& error);
    ct::Json serverInfo() const;
    ct::Json toolDefinition(const Tool& tool) const;
    bool knownSession(const ct::String& id) const;
    ct::String mintSession();

    ct::Unique<ct::HttpServer> mHttp;
    ct::Vector<Tool> mTools;
    ct::Vector<LogEntry> mLog;
    ct::Vector<ct::String> mSessions;
    ct::String mError;
    unsigned int mLogRevision;
    int mPort;
    bool mAllowEdits;
    bool mAllowCode;
};

} // namespace prisma3d
