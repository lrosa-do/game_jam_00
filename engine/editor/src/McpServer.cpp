#include "prisma3d/editor/McpServer.h"

#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/scene/SceneJson.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

namespace prisma3d
{

namespace
{

const char* const kVersionHeader = "MCP-Protocol-Version";
const char* const kSessionHeader = "Mcp-Session-Id";
const char* const kVersionMeta = "io.modelcontextprotocol/protocolVersion";
const char* const kCapabilitiesMeta = "io.modelcontextprotocol/clientCapabilities";
const char* const kServerInfoMeta = "io.modelcontextprotocol/serverInfo";
const char* const kBase64Prefix = "=?base64?";
const char* const kBase64Suffix = "?=";
const char* const kInstructions =
        "Tools for the prisma3d editor scene. Node paths are relative to the edited scene root "
        "('.' is the root). Angles are in degrees. Edits go through the editor's undo history, "
        "so the user can undo them with Ctrl+Z.";

ct::String lowered(const ct::String& text)
{
    ct::String result = text;
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        char& c = result.data()[i];
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + 32);
    }
    return result;
}

const ct::HttpHeader* findHeader(const ct::HttpRequest& request, const char* name)
{
    for (std::size_t i = 0; i < request.headers.size(); ++i)
        if (ct::detail::http_iequal(request.headers[i].name, name)) return &request.headers[i];
    return nullptr;
}

bool loopbackHost(const ct::String& host)
{
    const ct::String name = lowered(host);
    return name == "localhost" || name == "127.0.0.1";
}

bool splitAuthority(const ct::String& authority, ct::String& host)
{
    if (authority.empty()) return false;
    ct::String port;
    if (authority.front() == '[')
    {
        const std::size_t close = authority.find(']');
        if (close == ct::String::npos) return false;
        host = authority.substr(0, close + 1);
        if (close + 1 < authority.size())
        {
            if (authority[close + 1] != ':') return false;
            port = authority.substr(close + 2);
        }
    }
    else
    {
        const std::size_t colon = authority.rfind(':');
        if (colon == ct::String::npos) host = authority;
        else
        {
            host = authority.substr(0, colon);
            port = authority.substr(colon + 1);
        }
    }
    for (std::size_t i = 0; i < port.size(); ++i)
        if (port[i] < '0' || port[i] > '9') return false;
    return !host.empty();
}

ct::String shortened(const ct::String& text, std::size_t limit)
{
    if (text.size() <= limit) return text;
    std::size_t cut = limit;
    while (cut > 0 && (static_cast<unsigned char>(text[cut]) & 0xC0) == 0x80) --cut;
    return text.substr(0, cut) + "...";
}

ct::String clockText()
{
    const time_t now = time(nullptr);
    struct tm parts;
#if defined(_WIN32)
    localtime_s(&parts, &now);
#else
    localtime_r(&now, &parts);
#endif
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", parts.tm_hour, parts.tm_min, parts.tm_sec);
    return ct::String(buffer);
}

bool legacyVersion(const ct::String& version)
{
    for (std::size_t i = 0; i < McpServer::kLegacyVersionCount; ++i)
        if (version == McpServer::kLegacyVersions[i]) return true;
    return false;
}

bool randomBytes(unsigned char* out, std::size_t count)
{
    FILE* file = fopen("/dev/urandom", "rb");
    if (!file) return false;
    const std::size_t read = fread(out, 1, count, file);
    fclose(file);
    return read == count;
}

ct::String decodedHeader(const ct::String& value)
{
    const ct::String prefix(kBase64Prefix);
    const ct::String suffix(kBase64Suffix);
    if (!value.starts_with(prefix) || !value.ends_with(suffix) ||
            value.size() < prefix.size() + suffix.size())
        return value;
    const ct::String body =
            value.substr(prefix.size(), value.size() - prefix.size() - suffix.size());
    ct::Vector<std::uint8_t> bytes;
    if (!scenejson::decodeBase64(body, bytes)) return value;
    return ct::String(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

ct::Json supportedVersions()
{
    ct::Json list = ct::Json::array();
    list.push_back(ct::Json(McpServer::kModernVersion));
    for (std::size_t i = 0; i < McpServer::kLegacyVersionCount; ++i)
        list.push_back(ct::Json(McpServer::kLegacyVersions[i]));
    return list;
}

} // namespace

void McpToolResult::addText(const ct::String& text)
{
    ct::Json item = ct::Json::object();
    item["type"] = "text";
    item["text"] = text;
    content.push_back(item);
}

void McpToolResult::addImage(const ct::String& base64, const char* mimeType)
{
    ct::Json item = ct::Json::object();
    item["type"] = "image";
    item["data"] = base64;
    item["mimeType"] = mimeType;
    content.push_back(item);
}

void McpToolResult::fail(const ct::String& message)
{
    isError = true;
    addText(message);
}

McpServer::McpServer()
        : mLogRevision(0),
          mPort(0),
          mAllowEdits(false),
          mAllowCode(false)
{
}

McpServer::~McpServer()
{
    stop();
}

void McpServer::addTool(const char* name, const char* description, const ct::Json& schema,
        McpPermission level, const ToolHandler& handler)
{
    Tool tool;
    tool.name = name;
    tool.description = description;
    tool.schema = schema;
    tool.level = level;
    tool.handler = handler;
    mTools.push_back(tool);
}

std::size_t McpServer::toolCount() const
{
    return mTools.size();
}

const McpServer::Tool& McpServer::tool(std::size_t index) const
{
    return mTools[index];
}

const McpServer::Tool* McpServer::findTool(const ct::String& name) const
{
    for (std::size_t i = 0; i < mTools.size(); ++i)
        if (mTools[i].name == name) return &mTools[i];
    return nullptr;
}

std::size_t McpServer::listedToolCount() const
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < mTools.size(); ++i)
        if (permitted(mTools[i].level)) ++count;
    return count;
}

bool McpServer::start(int port)
{
    stop();
    mError = ct::String();
    if (port < 1 || port > 65535)
    {
        mError = "The port must be between 1 and 65535.";
        return false;
    }
    ct::Address address;
    if (!ct::Address::parse(kAddress, static_cast<std::uint16_t>(port), address))
    {
        mError = "Could not form the loopback address.";
        return false;
    }
    mHttp = ct::make_unique<ct::HttpServer>(16u);
    mHttp->route("*", kEndpoint, [this](const ct::HttpRequest& request, ct::HttpResponse& response)
            { handle(request, response); });
    ct::NetError failure = { nullptr, 0 };
    if (!mHttp->listen(address, &failure))
    {
        mError = ct::String("Could not listen on ") + kAddress + ":";
        mError.append_number(port);
        mError += ": ";
        mError += failure.message ? failure.message : "unknown error";
        mHttp.reset();
        return false;
    }
    mPort = port;
    return true;
}

void McpServer::stop()
{
    if (mHttp) mHttp->stop();
    mHttp.reset();
    mSessions.clear();
    mPort = 0;
}

bool McpServer::running() const
{
    return mHttp && mHttp->running();
}

int McpServer::port() const
{
    return mPort;
}

const ct::String& McpServer::error() const
{
    return mError;
}

void McpServer::poll()
{
    if (mHttp) mHttp->poll(0);
}

ct::String McpServer::endpointFor(int port)
{
    ct::String text = "http://";
    text += kAddress;
    text += ':';
    text.append_number(port);
    text += kEndpoint;
    return text;
}

ct::String McpServer::addCommandFor(int port)
{
    return ct::String("claude mcp add --transport http prisma3d ") + endpointFor(port);
}

ct::String McpServer::endpoint() const
{
    return endpointFor(mPort);
}

ct::String McpServer::addCommand() const
{
    return addCommandFor(mPort);
}

void McpServer::setAllowEdits(bool allow)
{
    mAllowEdits = allow;
}

bool McpServer::allowEdits() const
{
    return mAllowEdits;
}

void McpServer::setAllowCode(bool allow)
{
    mAllowCode = allow;
}

bool McpServer::allowCode() const
{
    return mAllowCode;
}

bool McpServer::permitted(McpPermission level) const
{
    switch (level)
    {
        case McpPermission::Read:
            return true;
        case McpPermission::Edit:
            return mAllowEdits;
        case McpPermission::Code:
            return mAllowCode;
    }
    return false;
}

const char* McpServer::permissionSwitch(McpPermission level)
{
    switch (level)
    {
        case McpPermission::Edit:
            return "Allow scene edits";
        case McpPermission::Code:
            return "Allow scripts & Play";
        default:
            return "";
    }
}

const ct::Vector<McpServer::LogEntry>& McpServer::log() const
{
    return mLog;
}

unsigned int McpServer::logRevision() const
{
    return mLogRevision;
}

void McpServer::clearLog()
{
    mLog.clear();
    ++mLogRevision;
}

std::size_t McpServer::sessionCount() const
{
    return mSessions.size();
}

bool McpServer::originAllowed(const ct::HttpRequest& request)
{
    const ct::HttpHeader* header = findHeader(request, "Origin");
    if (!header) return true;
    ct::String value = header->value;
    ct::String rest;
    if (value.starts_with("http://")) rest = value.substr(7);
    else if (value.starts_with("https://"))
        rest = value.substr(8);
    else
        return false;
    const std::size_t slash = rest.find('/');
    if (slash != ct::String::npos) rest = rest.substr(0, slash);
    ct::String host;
    return splitAuthority(rest, host) && loopbackHost(host);
}

bool McpServer::hostAllowed(const ct::HttpRequest& request)
{
    const ct::HttpHeader* header = findHeader(request, "Host");
    if (!header) return true;
    ct::String host;
    return splitAuthority(header->value, host) && loopbackHost(host);
}

ct::Json McpServer::serverInfo() const
{
    ct::Json info = ct::Json::object();
    info["name"] = kServerName;
    info["title"] = "prisma3d editor";
    info["version"] = EditorAbout::version();
    return info;
}

ct::Json McpServer::toolDefinition(const Tool& tool) const
{
    ct::Json definition = ct::Json::object();
    definition["name"] = tool.name;
    definition["description"] = tool.description;
    definition["inputSchema"] = tool.schema;
    ct::Json annotations = ct::Json::object();
    annotations["readOnlyHint"] = tool.level == McpPermission::Read;
    annotations["openWorldHint"] = false;
    definition["annotations"] = annotations;
    return definition;
}

bool McpServer::knownSession(const ct::String& id) const
{
    for (std::size_t i = 0; i < mSessions.size(); ++i)
        if (mSessions[i] == id) return true;
    return false;
}

ct::String McpServer::mintSession()
{
    unsigned char bytes[16] = {};
    if (!randomBytes(bytes, sizeof(bytes)))
    {
        std::uint64_t seed = time_nanos();
        for (std::size_t i = 0; i < sizeof(bytes); ++i)
        {
            seed = seed * 6364136223846793005ull + 1442695040888963407ull;
            bytes[i] = static_cast<unsigned char>(seed >> 56);
        }
    }
    static const char digits[] = "0123456789abcdef";
    ct::String id;
    for (std::size_t i = 0; i < sizeof(bytes); ++i)
    {
        id += digits[bytes[i] >> 4];
        id += digits[bytes[i] & 15];
    }
    if (mSessions.size() >= kSessionLimit) mSessions.erase(mSessions.begin());
    mSessions.push_back(id);
    return id;
}

void McpServer::record(const ct::String& tool, const ct::String& arguments, bool ok,
        const ct::String& outcome)
{
    LogEntry entry;
    entry.time = clockText();
    entry.tool = tool;
    entry.arguments = shortened(arguments, kArgumentsLimit);
    entry.ok = ok;
    entry.outcome = shortened(outcome, kArgumentsLimit);
    mLog.push_back(entry);
    while (mLog.size() > kLogLimit) mLog.erase(mLog.begin());
    ++mLogRevision;
}

void McpServer::reply(ct::HttpResponse& response, const ct::Json& id, ct::Json result,
        const Context& context)
{
    if (context.modern)
    {
        result["resultType"] = "complete";
        ct::Json meta = ct::Json::object();
        meta[kServerInfoMeta] = serverInfo();
        result["_meta"] = meta;
    }
    ct::Json envelope = ct::Json::object();
    envelope["jsonrpc"] = "2.0";
    envelope["id"] = id;
    envelope["result"] = result;
    response.status = 200;
    response.json(envelope.dump());
}

void McpServer::replyError(ct::HttpResponse& response, int status, const ct::Json* id, int code,
        const ct::String& message, const ct::Json* data)
{
    ct::Json error = ct::Json::object();
    error["code"] = code;
    error["message"] = message;
    if (data) error["data"] = *data;
    ct::Json envelope = ct::Json::object();
    envelope["jsonrpc"] = "2.0";
    envelope["id"] = id ? *id : ct::Json();
    envelope["error"] = error;
    response.status = status;
    response.json(envelope.dump());
}

void McpServer::handle(const ct::HttpRequest& request, ct::HttpResponse& response)
{
    if (!hostAllowed(request) || !originAllowed(request))
    {
        const ct::HttpHeader* origin = findHeader(request, "Origin");
        replyError(response, 403, nullptr, kInvalidRequest,
                "Forbidden: the Origin or Host header is not a loopback address.");
        record("(rejected)",
                origin ? origin->value : ct::String("Host ") + ct::String(request.header("Host")),
                false, "403 Forbidden origin");
        return;
    }
    if (!ct::detail::http_iequal(request.method, "POST"))
    {
        response.status = 405;
        response.set("Allow", "POST");
        response.text("This endpoint accepts POST only; it has no event stream.");
        record(request.method + " " + kEndpoint, ct::String(), false, "405 Method Not Allowed");
        return;
    }

    ct::Json::Error failure;
    const ct::Json message = ct::Json::parse(request.body, &failure);
    if (failure)
    {
        replyError(response, 400, nullptr, kParseError,
                ct::String("Parse error: ") + failure.message);
        record("(parse error)", ct::String(), false, failure.message);
        return;
    }
    if (!message.is_object())
    {
        replyError(response, 400, nullptr, kInvalidRequest,
                "The body must be a single JSON-RPC request or notification.");
        record("(invalid)", ct::String(), false, "not a JSON object");
        return;
    }
    dispatch(message, request, response);
}

bool McpServer::validateModern(const ct::Json& message, const ct::HttpRequest& request,
        ct::HttpResponse& response)
{
    const ct::Json* id = message.find("id");
    const ct::Json* params = message.find("params");
    const ct::Json* meta = params && params->is_object() ? params->find("_meta") : nullptr;
    const ct::Json* bodyVersion = meta && meta->is_object() ? meta->find(kVersionMeta) : nullptr;
    const ct::HttpHeader* header = findHeader(request, kVersionHeader);

    ct::String requested;
    if (bodyVersion && bodyVersion->is_string()) requested = bodyVersion->str();
    else if (header)
        requested = header->value;
    if (requested != kModernVersion)
    {
        ct::Json data = ct::Json::object();
        data["supported"] = supportedVersions();
        data["requested"] = requested;
        replyError(response, 400, id, kUnsupportedVersion, "Unsupported protocol version", &data);
        record(message["method"].as_cstr(), ct::String(), false, "unsupported protocol version");
        return false;
    }
    if (!bodyVersion || !bodyVersion->is_string())
    {
        replyError(response, 400, id, kInvalidParams,
                ct::String("Missing _meta field ") + kVersionMeta);
        record(message["method"].as_cstr(), ct::String(), false,
                "missing protocol version in _meta");
        return false;
    }
    if (!header)
    {
        replyError(response, 400, id, kHeaderMismatch,
                ct::String("Header mismatch: ") + kVersionHeader + " is missing");
        record(message["method"].as_cstr(), ct::String(), false, "missing protocol version header");
        return false;
    }
    if (header->value != bodyVersion->str())
    {
        replyError(response, 400, id, kHeaderMismatch,
                ct::String("Header mismatch: ") + kVersionHeader + " is '" + header->value +
                        "' but _meta says '" + bodyVersion->str() + "'");
        record(message["method"].as_cstr(), ct::String(), false,
                "protocol version header mismatch");
        return false;
    }
    const ct::Json* capabilities = meta->find(kCapabilitiesMeta);
    if (!capabilities || !capabilities->is_object())
    {
        replyError(response, 400, id, kInvalidParams,
                ct::String("Missing _meta field ") + kCapabilitiesMeta);
        record(message["method"].as_cstr(), ct::String(), false, "missing client capabilities");
        return false;
    }

    const ct::String method = message["method"].str();
    const ct::HttpHeader* methodHeader = findHeader(request, "Mcp-Method");
    if (!methodHeader || methodHeader->value != method)
    {
        replyError(response, 400, id, kHeaderMismatch,
                ct::String("Header mismatch: Mcp-Method must be '") + method + "'");
        record(method, ct::String(), false, "Mcp-Method header mismatch");
        return false;
    }
    if (method == "tools/call")
    {
        const ct::Json* name = params ? params->find("name") : nullptr;
        const ct::HttpHeader* nameHeader = findHeader(request, "Mcp-Name");
        if (name && name->is_string() &&
                (!nameHeader || decodedHeader(nameHeader->value) != name->str()))
        {
            replyError(response, 400, id, kHeaderMismatch,
                    ct::String("Header mismatch: Mcp-Name must be '") + name->str() + "'");
            record(method, ct::String(), false, "Mcp-Name header mismatch");
            return false;
        }
    }
    return true;
}

bool McpServer::validateLegacy(const ct::Json& message, const ct::HttpRequest& request,
        ct::HttpResponse& response)
{
    const ct::Json* id = message.find("id");
    const ct::String method = message["method"].str();
    if (method == "initialize") return true;

    const ct::HttpHeader* version = findHeader(request, kVersionHeader);
    if (version && !legacyVersion(version->value))
    {
        replyError(response, 400, id, kInvalidRequest,
                ct::String("Unsupported ") + kVersionHeader + ": " + version->value);
        record(method, ct::String(), false, "unsupported protocol version header");
        return false;
    }
    const ct::HttpHeader* session = findHeader(request, kSessionHeader);
    if (session && !knownSession(session->value))
    {
        replyError(response, 404, id, kInvalidRequest,
                "Unknown session. Send initialize again to start a new one.");
        record(method, ct::String(), false, "unknown session");
        return false;
    }
    return true;
}

void McpServer::dispatch(const ct::Json& message, const ct::HttpRequest& request,
        ct::HttpResponse& response)
{
    const ct::Json* methodNode = message.find("method");
    const ct::Json* id = message.find("id");
    if (!methodNode || !methodNode->is_string())
    {
        if (message.find("result") || message.find("error"))
        {
            response.status = 202;
            record("(response)", ct::String(), true, "ignored");
            return;
        }
        replyError(response, 400, id, kInvalidRequest, "The message has no method.");
        record("(invalid)", ct::String(), false, "no method");
        return;
    }
    const ct::String method = methodNode->str();
    if (!id)
    {
        response.status = 202;
        record(method, ct::String(), true, "accepted");
        return;
    }
    if (!id->is_string() && !id->is_number())
    {
        replyError(response, 400, nullptr, kInvalidRequest,
                "The request id must be a string or a number.");
        record(method, ct::String(), false, "bad request id");
        return;
    }

    const ct::Json* params = message.find("params");
    const ct::Json* meta = params && params->is_object() ? params->find("_meta") : nullptr;
    const bool bodyModern = meta && meta->is_object() && meta->find(kVersionMeta);
    const ct::HttpHeader* header = findHeader(request, kVersionHeader);
    const bool headerModern = header && !legacyVersion(header->value);
    Context context;
    context.modern = bodyModern || headerModern;

    if (context.modern)
    {
        if (!validateModern(message, request, response)) return;
        context.version = kModernVersion;
    }
    else
    {
        if (!validateLegacy(message, request, response)) return;
        context.version = header ? header->value : ct::String(kFallbackVersion);
    }

    if (method == "initialize" && !context.modern)
    {
        initialize(message, response);
        return;
    }
    if (method == "server/discover" && context.modern)
    {
        discover(message, response);
        return;
    }
    if (method == "ping" && !context.modern)
    {
        reply(response, *id, ct::Json::object(), context);
        record(method, ct::String(), true, "ok");
        return;
    }
    if (method == "tools/list")
    {
        listTools(message, context, response);
        return;
    }
    if (method == "tools/call")
    {
        callTool(message, context, response);
        return;
    }
    replyError(response, context.modern ? 404 : 200, id, kMethodNotFound,
            ct::String("Method not found: ") + method);
    record(method, ct::String(), false, "method not found");
}

void McpServer::initialize(const ct::Json& message, ct::HttpResponse& response)
{
    const ct::Json& id = message["id"];
    const ct::Json& params = message["params"];
    const ct::Json* requested = params.find("protocolVersion");
    ct::String version = kLegacyVersions[0];
    if (requested && requested->is_string() && legacyVersion(requested->str()))
        version = requested->str();

    ct::Json capabilities = ct::Json::object();
    ct::Json tools = ct::Json::object();
    tools["listChanged"] = false;
    capabilities["tools"] = tools;

    ct::Json result = ct::Json::object();
    result["protocolVersion"] = version;
    result["capabilities"] = capabilities;
    result["serverInfo"] = serverInfo();
    result["instructions"] = kInstructions;

    Context context;
    reply(response, id, result, context);
    response.set(kSessionHeader, mintSession());
    ct::String client = "client";
    const ct::Json* info = params.find("clientInfo");
    if (info && info->is_object()) client = (*info)["name"].as_cstr("client");
    record("initialize", client + " " + version, true, "session started");
}

void McpServer::discover(const ct::Json& message, ct::HttpResponse& response)
{
    ct::Json capabilities = ct::Json::object();
    ct::Json tools = ct::Json::object();
    tools["listChanged"] = false;
    capabilities["tools"] = tools;

    ct::Json versions = ct::Json::array();
    versions.push_back(ct::Json(kModernVersion));

    ct::Json result = ct::Json::object();
    result["supportedVersions"] = versions;
    result["capabilities"] = capabilities;
    result["instructions"] = kInstructions;
    result["ttlMs"] = 0;
    result["cacheScope"] = "private";

    Context context;
    context.modern = true;
    reply(response, message["id"], result, context);
    record("server/discover", ct::String(), true, "ok");
}

void McpServer::listTools(const ct::Json& message, const Context& context,
        ct::HttpResponse& response)
{
    ct::Json list = ct::Json::array();
    for (std::size_t i = 0; i < mTools.size(); ++i)
        if (permitted(mTools[i].level)) list.push_back(toolDefinition(mTools[i]));
    ct::Json result = ct::Json::object();
    result["tools"] = list;
    if (context.modern)
    {
        result["ttlMs"] = 0;
        result["cacheScope"] = "private";
    }
    reply(response, message["id"], result, context);
    ct::String outcome;
    outcome.append_number(static_cast<unsigned long long>(list.size()));
    outcome += " tools";
    record("tools/list", ct::String(), true, outcome);
}

bool McpServer::cleanArguments(const Tool& tool, const ct::Json& arguments, ct::Json& cleaned,
        ct::String& error)
{
    cleaned = ct::Json::object();
    const ct::Json& properties = tool.schema["properties"];
    const ct::Json::Object& members = arguments.members();
    ct::String valid;
    for (std::size_t i = 0; i < properties.members().size(); ++i)
    {
        if (i > 0) valid += ", ";
        valid += properties.members()[i].key;
    }
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        const ct::String& key = members[i].key;
        const ct::Json& value = members[i].value;
        const ct::Json* spec = properties.find(key);
        if (!spec)
        {
            error = ct::String("Unknown argument '") + key + "'. " +
                    (valid.empty() ? ct::String("This tool takes no arguments.")
                                   : ct::String("The arguments are: ") + valid + ".");
            return false;
        }
        if (value.is_null()) continue;
        const ct::String type = (*spec)["type"].as_cstr("");
        bool fits = true;
        if (type == "string") fits = value.is_string();
        else if (type == "boolean")
            fits = value.is_bool();
        else if (type == "number")
            fits = value.is_number();
        else if (type == "integer")
            fits = value.is_number() && value.as_double() == floor(value.as_double());
        else if (type == "object")
            fits = value.is_object();
        else if (type == "array")
        {
            fits = value.is_array();
            const ct::String itemType = (*spec)["items"]["type"].as_cstr("");
            for (std::size_t item = 0; fits && item < value.size(); ++item)
            {
                if (itemType == "string") fits = value[item].is_string();
                else if (itemType == "number")
                    fits = value[item].is_number();
            }
        }
        if (!fits)
        {
            error = ct::String("The argument '") + key + "' must be of type " + type + ".";
            return false;
        }
        const ct::Json& choices = (*spec)["enum"];
        if (choices.is_array() && value.is_string())
        {
            bool found = false;
            ct::String names;
            for (std::size_t c = 0; c < choices.size(); ++c)
            {
                if (value.str() == choices[c].as_cstr("")) found = true;
                if (c > 0) names += ", ";
                names += choices[c].as_cstr("");
            }
            if (!found)
            {
                error = ct::String("The argument '") + key + "' must be one of: " + names + ".";
                return false;
            }
        }
        cleaned[key] = value;
    }
    const ct::Json* required = tool.schema.find("required");
    for (std::size_t i = 0; required && i < required->size(); ++i)
    {
        const ct::String name = (*required)[i].as_cstr("");
        if (!cleaned.find(name))
        {
            error = ct::String("Missing required argument '") + name + "'.";
            return false;
        }
    }
    return true;
}

void McpServer::callTool(const ct::Json& message, const Context& context,
        ct::HttpResponse& response)
{
    const ct::Json& id = message["id"];
    const ct::Json* params = message.find("params");
    const ct::Json* name = params && params->is_object() ? params->find("name") : nullptr;
    const int failure = context.modern ? 400 : 200;
    if (!name || !name->is_string())
    {
        replyError(response, failure, &id, kInvalidParams, "tools/call needs a tool name.");
        record("tools/call", ct::String(), false, "missing tool name");
        return;
    }
    const ct::Json* arguments = params->find("arguments");
    if (arguments && !arguments->is_null() && !arguments->is_object())
    {
        replyError(response, failure, &id, kInvalidParams, "The tool arguments must be an object.");
        record(name->str(), ct::String(), false, "arguments are not an object");
        return;
    }
    const ct::Json empty = ct::Json::object();
    const ct::Json& values = arguments && arguments->is_object() ? *arguments : empty;
    const ct::String argumentText = values.dump();

    const Tool* tool = findTool(name->str());
    if (!tool)
    {
        replyError(response, failure, &id, kInvalidParams,
                ct::String("Unknown tool: ") + name->str());
        record(name->str(), argumentText, false, "unknown tool");
        return;
    }

    McpToolResult result;
    ct::Json cleaned;
    ct::String problem;
    if (!permitted(tool->level))
        result.fail(ct::String("The tool '") + tool->name +
                    "' is turned off. Ask the user to turn on '" + permissionSwitch(tool->level) +
                    "' in the AI panel of the prisma3d editor, then call it again.");
    else if (!cleanArguments(*tool, values, cleaned, problem))
        result.fail(problem);
    else
        tool->handler(cleaned, result);

    ct::Json body = ct::Json::object();
    body["content"] = result.content;
    body["isError"] = result.isError;
    reply(response, id, body, context);

    ct::String outcome = "ok";
    if (result.isError)
    {
        outcome = "error";
        if (result.content.size() > 0)
        {
            const ct::Json* text = result.content[0].find("text");
            if (text && text->is_string()) outcome = text->str();
        }
    }
    record(tool->name, argumentText, !result.isError, outcome);
}

} // namespace prisma3d
