#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/McpPanel.h"
#include "prisma3d/editor/McpServer.h"
#include "prisma3d/editor/McpTools.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/http_client.hpp>
#include <ct/json.hpp>
#include <ct/socket.hpp>
#include <ct/thread.hpp>

#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const int kFirstPort = 38700;

bool useVulkan = false;
const char* gOnly = nullptr;

bool wanted(const char* name)
{
    return !gOnly || strcmp(gOnly, name) == 0;
}

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_mcp_test_vulkan" : "/editor_mcp_test";
    return root;
}

InputEvent key(Key which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = modifiers;
    return event;
}

InputEvent motion(int x, int y)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    event.mouseMotion.buttonMask = 0;
    return event;
}

InputEvent button(int x, int y, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = MouseButton::Left;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = 0;
    return event;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorShell& shell;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    InspectorPanel& inspector;
    EditorDocument& document;
    EditorSettings& settings;
    ProjectUi& ui;
    EditorViewportPanel& panel;
    HierarchyActions& actions;
    EditorMenus& menus;
    EditorStatusBar& status;
    AssetsPanel& assets;
    MaterialPanel& materials;
    GamePanel& game;
    EditorPlay& play;
    McpServer& server;
    McpPanel& mcp;
    int sceneCounter = 0;

    bool send(const InputEvent& event)
    {
        if (play.handleEvent(event)) return true;
        if (ui.handleEvent(event)) return true;
        if (actions.handleEvent(event)) return true;
        host.handleEvent(event);
        panel.handleEvent(event);
        return false;
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        mcp.update();
        selection.update();
        selection.flush();
        hierarchy.update();
        inspector.update();
        ui.update();
        play.update();
        actions.update();
        menus.update();
        status.update(delta);
        assets.update(delta);
        materials.update();
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }

    bool tap(Key which, unsigned modifiers = 0)
    {
        const bool consumed = send(key(which, true, modifiers));
        frames(2);
        send(key(which, false, modifiers));
        frames(2);
        return consumed;
    }

    void click(int x, int y)
    {
        send(motion(x, y));
        frames(2);
        send(button(x, y, true));
        frames(2);
        send(button(x, y, false));
        frames(3);
    }

    EditorUndoRedoManager& history()
    {
        return document.undoRedo();
    }

    bool undoKey()
    {
        return tap(Key::Z, kModControl);
    }

    bool redoKey()
    {
        return tap(Key::Y, kModControl);
    }

    void fresh()
    {
        if (play.playing()) play.stop();
        frames(2);
        CHECK(document.newScene());
        frames(3);
    }
};

struct Reply
{
    bool sent = false;
    int status = 0;
    ct::String body;
    ct::Json json;
    ct::HttpResponse response;

    ct::String header(const char* name) const
    {
        return ct::String(response.header(name));
    }
};

struct Headers
{
    ct::Vector<ct::HttpHeader> list;

    Headers& add(const char* name, const ct::String& value)
    {
        for (std::size_t i = 0; i < list.size(); ++i)
        {
            if (ct::detail::http_iequal(list[i].name, name))
            {
                list[i].value = value;
                return *this;
            }
        }
        list.push_back(ct::HttpHeader{ ct::String(name), value });
        return *this;
    }

    Headers& remove(const char* name)
    {
        for (std::size_t i = 0; i < list.size(); ++i)
        {
            if (ct::detail::http_iequal(list[i].name, name))
            {
                list.erase(list.begin() + i);
                return *this;
            }
        }
        return *this;
    }
};

Reply exchange(Rig& rig, int port, const char* method, const ct::String& body,
        const Headers& headers)
{
    Reply reply;
    ct::Atomic<int> finished(0);
    ct::HttpRequest request;
    request.method = method;
    request.path = "/mcp";
    request.headers = headers.list;
    request.body = body;
    ct::Address address;
    ct::Address::parse("127.0.0.1", static_cast<std::uint16_t>(port), address);
    bool sent = false;
    ct::HttpResponse response;
    {
        ct::Thread worker(
                [&]
                {
                    sent = ct::HttpClient::request(request, address, response, nullptr, 20000);
                    finished.store(1);
                });
        for (int guard = 0; guard < 6000 && !finished.load(); ++guard) rig.frame();
        worker.join();
    }
    reply.sent = sent;
    reply.response = response;
    reply.status = sent ? response.status : 0;
    reply.body = response.body;
    if (sent && !response.body.empty()) reply.json = ct::Json::parse(response.body);
    return reply;
}

struct Client
{
    Rig& rig;
    int port;
    ct::String session;
    ct::String version;
    int nextId = 1;

    Client(Rig& owner, int listening)
            : rig(owner),
              port(listening)
    {
    }

    Headers headers() const
    {
        Headers result;
        ct::String host = "127.0.0.1:";
        host.append_number(port);
        result.add("Host", host);
        result.add("Content-Type", "application/json");
        result.add("Accept", "application/json, text/event-stream");
        if (!version.empty()) result.add("MCP-Protocol-Version", version);
        if (!session.empty()) result.add("Mcp-Session-Id", session);
        return result;
    }

    Reply post(const ct::Json& message, const Headers& extra)
    {
        return exchange(rig, port, "POST", message.dump(), extra);
    }

    ct::Json envelope(const char* method, const ct::Json& params, bool notification = false)
    {
        ct::Json message = ct::Json::object();
        message["jsonrpc"] = "2.0";
        if (!notification) message["id"] = nextId++;
        message["method"] = method;
        if (!params.is_null()) message["params"] = params;
        return message;
    }

    Reply rpc(const char* method, const ct::Json& params = ct::Json())
    {
        return post(envelope(method, params), headers());
    }

    bool initialize(const char* protocol = "2025-11-25")
    {
        ct::Json params = ct::Json::object();
        params["protocolVersion"] = protocol;
        params["capabilities"] = ct::Json::object();
        ct::Json info = ct::Json::object();
        info["name"] = "test-client";
        info["version"] = "1.0";
        params["clientInfo"] = info;
        const Reply reply = rpc("initialize", params);
        if (!reply.sent || reply.status != 200 || !reply.json["result"].is_object()) return false;
        session = reply.header("Mcp-Session-Id");
        version = reply.json["result"]["protocolVersion"].as_cstr("");
        const Reply notified =
                post(envelope("notifications/initialized", ct::Json(), true), headers());
        return notified.status == 202;
    }

    ct::Json tool(const char* name, const ct::Json& arguments = ct::Json::object())
    {
        ct::Json params = ct::Json::object();
        params["name"] = name;
        params["arguments"] = arguments;
        const Reply reply = rpc("tools/call", params);
        if (!reply.sent || reply.status != 200) return ct::Json();
        return reply.json["result"];
    }
};

bool isError(const ct::Json& result)
{
    return result.is_object() && result["isError"].as_bool(false);
}

bool succeeded(const ct::Json& result)
{
    return result.is_object() && result["content"].is_array() && !result["isError"].as_bool(false);
}

ct::String textOf(const ct::Json& result)
{
    if (!result.is_object() || !result["content"].is_array()) return ct::String();
    for (std::size_t i = 0; i < result["content"].size(); ++i)
    {
        const ct::Json& item = result["content"][i];
        if (item["type"].as_cstr("") == ct::String("text")) return item["text"].as_cstr("");
    }
    return ct::String();
}

ct::Json dataOf(const ct::Json& result)
{
    return ct::Json::parse(textOf(result));
}

ct::Json args()
{
    return ct::Json::object();
}

ct::Json vec(double x, double y, double z)
{
    ct::Json value = ct::Json::array();
    value.push_back(ct::Json(x));
    value.push_back(ct::Json(y));
    value.push_back(ct::Json(z));
    return value;
}

ct::Json color(double r, double g, double b, double a = 1.0)
{
    ct::Json value = vec(r, g, b);
    value.push_back(ct::Json(a));
    return value;
}

bool hasTool(const ct::Json& list, const char* name)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i]["name"].as_cstr("") == ct::String(name)) return true;
    return false;
}

Node* nodeAt(Rig& rig, const char* path)
{
    Node* scene = rig.document.scene();
    return scene ? scene->getNode(path) : nullptr;
}

bool nearly(double a, double b, double tolerance = 1e-3)
{
    return fabs(a - b) <= tolerance;
}

int findFreePort(McpServer& server)
{
    for (int port = kFirstPort; port < kFirstPort + 400; ++port)
    {
        if (!server.start(port)) continue;
        server.stop();
        return port;
    }
    return 0;
}

bool startServer(Rig& rig, int& port)
{
    port = findFreePort(rig.server);
    if (!port) return false;
    rig.settings.mcpPort = port;
    rig.mcp.portInput()->setText(ct::String());
    ct::String text;
    text.append_number(port);
    rig.mcp.portInput()->setText(text);
    return rig.mcp.start();
}

bool decodePng(const ct::Json& result, int& width, int& height, ct::Vector<std::uint8_t>& bytes)
{
    if (!result.is_object() || !result["content"].is_array()) return false;
    for (std::size_t i = 0; i < result["content"].size(); ++i)
    {
        const ct::Json& item = result["content"][i];
        if (item["type"].as_cstr("") != ct::String("image")) continue;
        if (item["mimeType"].as_cstr("") != ct::String("image/png")) return false;
        if (!scenejson::decodeBase64(item["data"].as_cstr(""), bytes)) return false;
        if (bytes.size() < 24) return false;
        static const std::uint8_t signature[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
        for (int b = 0; b < 8; ++b)
            if (bytes[b] != signature[b]) return false;
        width = (bytes[16] << 24) | (bytes[17] << 16) | (bytes[18] << 8) | bytes[19];
        height = (bytes[20] << 24) | (bytes[21] << 16) | (bytes[22] << 8) | bytes[23];
        return width > 0 && height > 0;
    }
    return false;
}

bool imageHasDetail(const ct::Vector<std::uint8_t>& png)
{
    Pixmap image;
    if (!image.load_from_memory(png.data(), static_cast<std::uint32_t>(png.size()))) return false;
    const int components = image.components;
    if (components < 3 || image.width <= 0 || image.height <= 0) return false;
    int different = 0;
    const unsigned char* first = image.pixels;
    for (int y = 0; y < image.height; y += 3)
    {
        for (int x = 0; x < image.width; x += 3)
        {
            const unsigned char* pixel = image.pixels + (y * image.width + x) * components;
            if (abs(pixel[0] - first[0]) + abs(pixel[1] - first[1]) + abs(pixel[2] - first[2]) > 24)
                ++different;
        }
    }
    return different > 40;
}

int countPixels(const ct::Vector<std::uint8_t>& png, bool (*match)(int, int, int))
{
    Pixmap image;
    if (!image.load_from_memory(png.data(), static_cast<std::uint32_t>(png.size()))) return 0;
    int count = 0;
    for (int y = 0; y < image.height; ++y)
    {
        for (int x = 0; x < image.width; ++x)
        {
            const unsigned char* pixel = image.pixels + (y * image.width + x) * image.components;
            if (match(pixel[0], pixel[1], pixel[2])) ++count;
        }
    }
    return count;
}

bool orangeish(int r, int g, int b)
{
    return r > 180 && g > 70 && r > g + 50 && r > b + 90;
}

bool greenish(int r, int g, int b)
{
    return g > r + 15 && g > b + 15;
}

void testDefaults(Rig& rig)
{
    CHECK(!rig.server.running());
    CHECK(rig.server.port() == 0);
    CHECK(rig.server.error().empty());
    CHECK(!rig.server.allowEdits());
    CHECK(!rig.server.allowCode());
    CHECK(rig.mcp.statusText() == "Stopped");
    CHECK(rig.mcp.statusLabel()->text() == "Stopped");
    CHECK(!rig.mcp.serverSwitch()->isOn());
    CHECK(!rig.mcp.editsSwitch()->isOn());
    CHECK(!rig.mcp.codeSwitch()->isOn());
    CHECK(rig.mcp.portInput()->text() == "8787");
    CHECK(EditorSettings().mcpPort == McpServer::kDefaultPort);
    CHECK(!EditorSettings().mcpAllowEdits);
    CHECK(!EditorSettings().mcpAllowCode);
    CHECK(rig.mcp.logGrid() != nullptr);
    CHECK(McpServer::addCommandFor(8787) ==
            "claude mcp add --transport http prisma3d http://127.0.0.1:8787/mcp");
    CHECK(rig.server.toolCount() == 27);
    CHECK(rig.server.listedToolCount() == 8);

    ct::String names;
    for (std::size_t i = 0; i < rig.server.toolCount(); ++i)
    {
        const McpServer::Tool& tool = rig.server.tool(i);
        CHECK(!tool.description.empty());
        CHECK(tool.schema["type"].as_cstr("") == ct::String("object"));
        CHECK(tool.schema["properties"].is_object());
        for (std::size_t j = 0; j < i; ++j) CHECK(!(rig.server.tool(j).name == tool.name));
        for (std::size_t m = 0; m < tool.schema["properties"].members().size(); ++m)
            CHECK(tool.schema["properties"].members()[m].value["description"].is_string());
    }
}

void testProtocol(Rig& rig)
{
    int port = 0;
    CHECK(startServer(rig, port));
    CHECK(rig.server.running());
    CHECK(rig.server.port() == port);
    CHECK(rig.mcp.serverSwitch()->isOn());
    CHECK(rig.mcp.statusLabel()->text().contains("Listening on 127.0.0.1:"));
    CHECK(rig.mcp.statusLabel()->text().contains("/mcp"));
    CHECK(!rig.mcp.portInput()->isEnabled());

    Client client(rig, port);

    const Reply get = exchange(rig, port, "GET", ct::String(), client.headers());
    CHECK(get.sent && get.status == 405);
    CHECK(get.header("Allow") == "POST");
    const Reply del = exchange(rig, port, "DELETE", ct::String(), client.headers());
    CHECK(del.sent && del.status == 405);

    ct::Json params = ct::Json::object();
    params["protocolVersion"] = "2025-06-18";
    params["capabilities"] = ct::Json::object();
    ct::Json info = ct::Json::object();
    info["name"] = "probe";
    info["version"] = "0";
    params["clientInfo"] = info;
    const Reply init = client.rpc("initialize", params);
    CHECK(init.sent && init.status == 200);
    CHECK(init.json["jsonrpc"].as_cstr("") == ct::String("2.0"));
    CHECK(init.json["id"].as_int() == 1);
    CHECK(init.json["result"]["protocolVersion"].as_cstr("") == ct::String("2025-06-18"));
    CHECK(init.json["result"]["serverInfo"]["name"].as_cstr("") == ct::String("prisma3d-editor"));
    CHECK(init.json["result"]["capabilities"]["tools"].is_object());
    CHECK(!init.json["result"]["capabilities"]["tools"]["listChanged"].as_bool(true));
    CHECK(!init.json["result"]["capabilities"].contains("resources"));
    CHECK(!init.json["result"]["capabilities"].contains("prompts"));
    CHECK(init.header("Mcp-Session-Id").size() == 32);
    CHECK(rig.server.sessionCount() == 1);

    params["protocolVersion"] = "1999-01-01";
    const Reply old = client.rpc("initialize", params);
    CHECK(old.status == 200);
    CHECK(old.json["result"]["protocolVersion"].as_cstr("") == ct::String("2025-11-25"));
    CHECK(old.header("Mcp-Session-Id") != init.header("Mcp-Session-Id"));

    CHECK(client.initialize());
    CHECK(client.session.size() == 32);
    CHECK(client.version == "2025-11-25");

    const Reply initialized = client.post(
            client.envelope("notifications/initialized", ct::Json(), true), client.headers());
    CHECK(initialized.status == 202);
    CHECK(initialized.body.empty());

    const Reply ping = client.rpc("ping");
    CHECK(ping.status == 200 && ping.json["result"].is_object());

    const Reply missing = client.rpc("resources/list");
    CHECK(missing.status == 200);
    CHECK(missing.json["error"]["code"].as_int() == -32601);
    CHECK(missing.json["id"].as_int() == client.nextId - 1);
    CHECK(client.rpc("prompts/list").json["error"]["code"].as_int() == -32601);

    const Reply list = client.rpc("tools/list");
    CHECK(list.status == 200);
    CHECK(list.json["result"]["tools"].is_array());
    CHECK(list.json["result"]["tools"].size() == 8);
    CHECK(!list.json["result"].contains("resultType"));
    const ct::Json& first = list.json["result"]["tools"][0];
    CHECK(first["name"].is_string() && first["description"].is_string());
    CHECK(first["inputSchema"]["type"].as_cstr("") == ct::String("object"));
    CHECK(first["annotations"]["readOnlyHint"].as_bool(false));

    Headers badOrigin = client.headers();
    badOrigin.add("Origin", "http://evil.example");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);
    badOrigin.add("Origin", "http://127.0.0.1.evil.example:8787");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);
    badOrigin.add("Origin", "null");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);
    badOrigin.add("Origin", "https://localhost.evil.example");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);
    badOrigin.add("Origin", "file:///tmp/page.html");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);

    badOrigin.add("Origin", "http://[::1]:3000");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), badOrigin).status == 403);
    const char* const goodOrigins[] = { "http://localhost:5173", "http://127.0.0.1",
        "https://localhost" };
    for (const char* origin: goodOrigins)
    {
        Headers good = client.headers();
        good.add("Origin", origin);
        CHECK(client.post(client.envelope("tools/list", ct::Json()), good).status == 200);
    }
    Headers rebound = client.headers();
    rebound.add("Host", "rebind.example:8787");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), rebound).status == 403);

    Headers staleSession = client.headers();
    staleSession.add("Mcp-Session-Id", "00000000000000000000000000000000");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), staleSession).status == 404);
    Headers badVersion = client.headers();
    badVersion.add("MCP-Protocol-Version", "2025-11-26");
    const Reply badVersionReply =
            client.post(client.envelope("tools/list", ct::Json()), badVersion);
    CHECK(badVersionReply.status == 400 || badVersionReply.json["error"]["code"].is_number());

    Headers noVersion = client.headers();
    noVersion.remove("MCP-Protocol-Version");
    CHECK(client.post(client.envelope("tools/list", ct::Json()), noVersion).status == 200);

    const Reply garbage = exchange(rig, port, "POST", "{not json", client.headers());
    CHECK(garbage.status == 400);
    CHECK(garbage.json["error"]["code"].as_int() == -32700);
    CHECK(garbage.json["id"].is_null());
    const Reply batch = exchange(rig, port, "POST", "[]", client.headers());
    CHECK(batch.status == 400);
    CHECK(batch.json["error"]["code"].as_int() == -32600);
    const Reply nullId = exchange(rig, port, "POST",
            "{\"jsonrpc\":\"2.0\",\"id\":null,\"method\":\"tools/list\"}", client.headers());
    CHECK(nullId.status == 400);

    ct::Json call = ct::Json::object();
    call["name"] = "no_such_tool";
    const Reply unknown = client.rpc("tools/call", call);
    CHECK(unknown.json["error"]["code"].as_int() == -32602);
    const Reply noName = client.rpc("tools/call", ct::Json::object());
    CHECK(noName.json["error"]["code"].as_int() == -32602);
    call["name"] = "get_selection";
    call["arguments"] = "text";
    CHECK(client.rpc("tools/call", call).json["error"]["code"].as_int() == -32602);

    Client modern(rig, port);
    ct::Json meta = ct::Json::object();
    meta["io.modelcontextprotocol/protocolVersion"] = "2026-07-28";
    meta["io.modelcontextprotocol/clientCapabilities"] = ct::Json::object();
    ct::Json clientInfo = ct::Json::object();
    clientInfo["name"] = "modern";
    clientInfo["version"] = "1";
    meta["io.modelcontextprotocol/clientInfo"] = clientInfo;
    ct::Json modernParams = ct::Json::object();
    modernParams["_meta"] = meta;

    Headers modernHeaders = modern.headers();
    modernHeaders.add("MCP-Protocol-Version", "2026-07-28");
    modernHeaders.add("Mcp-Method", "server/discover");
    const Reply discover =
            modern.post(modern.envelope("server/discover", modernParams), modernHeaders);
    CHECK(discover.status == 200);
    CHECK(discover.json["result"]["resultType"].as_cstr("") == ct::String("complete"));
    CHECK(discover.json["result"]["supportedVersions"][0].as_cstr("") == ct::String("2026-07-28"));
    CHECK(discover.json["result"]["capabilities"]["tools"].is_object());
    CHECK(discover.json["result"]["_meta"]["io.modelcontextprotocol/serverInfo"]["name"]
                    .is_string());
    CHECK(discover.json["result"]["ttlMs"].as_int(-1) == 0);
    CHECK(!discover.header("Mcp-Session-Id").size());

    modernHeaders.add("Mcp-Method", "tools/list");
    const Reply modernList =
            modern.post(modern.envelope("tools/list", modernParams), modernHeaders);
    CHECK(modernList.status == 200);
    CHECK(modernList.json["result"]["resultType"].as_cstr("") == ct::String("complete"));
    CHECK(modernList.json["result"]["tools"].size() == 8);
    CHECK(modernList.json["result"]["ttlMs"].as_int(-1) == 0);
    CHECK(modernList.json["result"]["cacheScope"].is_string());

    ct::Json modernCall = ct::Json::object();
    modernCall["name"] = "get_selection";
    modernCall["arguments"] = ct::Json::object();
    modernCall["_meta"] = meta;
    Headers callHeaders = modernHeaders;
    callHeaders.add("Mcp-Method", "tools/call");
    callHeaders.add("Mcp-Name", "get_selection");
    const Reply modernResult = modern.post(modern.envelope("tools/call", modernCall), callHeaders);
    CHECK(modernResult.status == 200);
    CHECK(modernResult.json["result"]["resultType"].as_cstr("") == ct::String("complete"));
    CHECK(succeeded(modernResult.json["result"]));
    callHeaders.add("Mcp-Name", "get_node");
    const Reply mismatch = modern.post(modern.envelope("tools/call", modernCall), callHeaders);
    CHECK(mismatch.status == 400);
    CHECK(mismatch.json["error"]["code"].as_int() == -32020);
    callHeaders.add("Mcp-Name", "=?base64?Z2V0X3NlbGVjdGlvbg==?=");
    CHECK(modern.post(modern.envelope("tools/call", modernCall), callHeaders).status == 200);

    Headers noMethod = callHeaders;
    noMethod.remove("Mcp-Method");
    const Reply noMethodReply = modern.post(modern.envelope("tools/call", modernCall), noMethod);
    CHECK(noMethodReply.status == 400);
    CHECK(noMethodReply.json["error"]["code"].as_int() == -32020);
    Headers noProtocol = callHeaders;
    noProtocol.remove("MCP-Protocol-Version");
    CHECK(modern.post(modern.envelope("tools/call", modernCall), noProtocol)
                    .json["error"]["code"]
                    .as_int() == -32020);

    ct::Json future = modernParams;
    future["_meta"]["io.modelcontextprotocol/protocolVersion"] = "2099-01-01";
    Headers futureHeaders = modernHeaders;
    futureHeaders.add("MCP-Protocol-Version", "2099-01-01");
    const Reply unsupported = modern.post(modern.envelope("tools/list", future), futureHeaders);
    CHECK(unsupported.status == 400);
    CHECK(unsupported.json["error"]["code"].as_int() == -32022);
    CHECK(unsupported.json["error"]["data"]["requested"].as_cstr("") == ct::String("2099-01-01"));
    CHECK(unsupported.json["error"]["data"]["supported"].size() == 5);

    ct::Json noCapabilities = modernParams;
    noCapabilities["_meta"].erase("io.modelcontextprotocol/clientCapabilities");
    CHECK(modern.post(modern.envelope("tools/list", noCapabilities), modernHeaders).status == 400);

    modernHeaders.add("Mcp-Method", "resources/list");
    const Reply modernMissing =
            modern.post(modern.envelope("resources/list", modernParams), modernHeaders);
    CHECK(modernMissing.status == 404);
    CHECK(modernMissing.json["error"]["code"].as_int() == -32601);
    modernHeaders.add("Mcp-Method", "ping");
    CHECK(modern.post(modern.envelope("ping", modernParams), modernHeaders)
                    .json["error"]["code"]
                    .as_int() == -32601);

    CHECK(rig.server.log().size() > 20);
    bool sawRejected = false;
    bool sawInitialize = false;
    for (std::size_t i = 0; i < rig.server.log().size(); ++i)
    {
        const McpServer::LogEntry& entry = rig.server.log()[i];
        if (entry.tool == "(rejected)" && !entry.ok) sawRejected = true;
        if (entry.tool == "initialize" && entry.ok) sawInitialize = true;
        CHECK(entry.time.size() == 8);
    }
    CHECK(sawRejected && sawInitialize);

    rig.mcp.stop();
    rig.frames(2);
    CHECK(!rig.server.running());
    CHECK(rig.mcp.statusLabel()->text() == "Stopped");
    CHECK(!rig.mcp.serverSwitch()->isOn());
    CHECK(rig.mcp.portInput()->isEnabled());
    const Reply closed = exchange(rig, port, "POST", "{}", client.headers());
    CHECK(!closed.sent);
}

void testPermissions(Rig& rig)
{
    int port = 0;
    CHECK(startServer(rig, port));
    Client client(rig, port);
    CHECK(client.initialize());
    rig.fresh();

    ct::Json list = client.rpc("tools/list").json["result"]["tools"];
    CHECK(list.size() == 8);
    CHECK(hasTool(list, "get_scene_tree"));
    CHECK(hasTool(list, "screenshot"));
    CHECK(!hasTool(list, "create_node"));
    CHECK(!hasTool(list, "write_script"));
    CHECK(!hasTool(list, "play"));

    const ct::Json refused = client.tool("create_node",
            []
            {
                ct::Json a = ct::Json::object();
                a["class"] = "Node3D";
                return a;
            }());
    CHECK(isError(refused));
    CHECK(textOf(refused).contains("Allow scene edits"));
    CHECK(rig.document.scene()->getChildCount() == 0);
    CHECK(!rig.document.dirty());
    const ct::Json refusedCode = client.tool("play");
    CHECK(isError(refusedCode));
    CHECK(textOf(refusedCode).contains("Allow scripts & Play"));
    CHECK(!rig.play.playing());

    rig.mcp.editsSwitch()->setOn(true);
    CHECK(rig.server.allowEdits());
    CHECK(rig.settings.mcpAllowEdits);
    {
        ct::Json made = args();
        made["class"] = "Node3D";
        made["name"] = "Holder";
        CHECK(succeeded(client.tool("create_node", made)));
        ct::Json scripted = args();
        scripted["path"] = "Holder";
        ct::Json values = args();
        values["script"] = "Assets/Scripts/x.py";
        scripted["properties"] = values;
        const ct::Json blocked = client.tool("set_properties", scripted);
        CHECK(isError(blocked));
        CHECK(textOf(blocked).contains("Allow scripts & Play"));
        made["name"] = "Another";
        made["properties"] = values;
        CHECK(isError(client.tool("create_node", made)));
        CHECK(nodeAt(rig, "Another") == nullptr);
        CHECK(nodeAt(rig, "Holder")->getScript().empty());
    }
    list = client.rpc("tools/list").json["result"]["tools"];
    CHECK(list.size() == 23);
    CHECK(hasTool(list, "create_node"));
    CHECK(hasTool(list, "create_environment"));
    CHECK(!hasTool(list, "write_script"));
    CHECK(!list[8]["annotations"]["readOnlyHint"].as_bool(true));
    const ct::Json stillRefused = client.tool("attach_script", args());
    CHECK(isError(stillRefused));
    CHECK(textOf(stillRefused).contains("Allow scripts & Play"));

    rig.mcp.codeSwitch()->setOn(true);
    CHECK(rig.server.allowCode());
    CHECK(rig.settings.mcpAllowCode);
    list = client.rpc("tools/list").json["result"]["tools"];
    CHECK(list.size() == 27);
    CHECK(rig.server.listedToolCount() == list.size());
    CHECK(hasTool(list, "write_script"));
    CHECK(hasTool(list, "play"));
    CHECK(hasTool(list, "stop"));

    rig.mcp.editsSwitch()->setOn(false);
    list = client.rpc("tools/list").json["result"]["tools"];
    CHECK(!hasTool(list, "create_node"));
    CHECK(hasTool(list, "play"));
    CHECK(isError(client.tool("create_node", args())));
    rig.mcp.codeSwitch()->setOn(false);
    list = client.rpc("tools/list").json["result"]["tools"];
    CHECK(list.size() == 8);
    CHECK(!rig.settings.mcpAllowEdits && !rig.settings.mcpAllowCode);

    rig.mcp.stop();
}

void testReadTools(Rig& rig, Client& client)
{
    rig.fresh();
    const ct::Json tree = dataOf(client.tool("get_scene_tree"));
    CHECK(tree["scene"]["project_open"].as_bool(false));
    CHECK(!tree["scene"]["playing"].as_bool(true));
    CHECK(tree["tree"]["path"].as_cstr("") == ct::String("."));
    CHECK(tree["tree"]["class"].as_cstr("") == ct::String("Node3D"));
    CHECK(!tree["tree"].contains("children"));

    Node3D* a = rig.document.scene()->createChild<Node3D>();
    a->setName("A");
    Node3D* b = a->createChild<Node3D>();
    b->setName("B");
    rig.frames(2);
    const ct::Json deeper = dataOf(client.tool("get_scene_tree"));
    CHECK(deeper["tree"]["children"][0]["path"].as_cstr("") == ct::String("A"));
    CHECK(deeper["tree"]["children"][0]["children"][0]["path"].as_cstr("") == ct::String("A/B"));
    ct::Json limited = args();
    limited["max_depth"] = 0;
    const ct::Json shallow = dataOf(client.tool("get_scene_tree", limited));
    CHECK(shallow["tree"]["children_not_shown"].as_int() == 1);
    ct::Json subtree = args();
    subtree["path"] = "A";
    CHECK(dataOf(client.tool("get_scene_tree", subtree))["tree"]["name"].as_cstr("") ==
            ct::String("A"));
    subtree["path"] = "A/B";
    CHECK(dataOf(client.tool("get_scene_tree", subtree))["tree"]["path"].as_cstr("") ==
            ct::String("A/B"));
    subtree["path"] = "Missing";
    CHECK(isError(client.tool("get_scene_tree", subtree)));

    ct::Json node = args();
    node["path"] = "A/B";
    const ct::Json detail = dataOf(client.tool("get_node", node));
    CHECK(detail["name"].as_cstr("") == ct::String("B"));
    CHECK(detail["class"].as_cstr("") == ct::String("Node3D"));
    CHECK(detail["parent"].as_cstr("") == ct::String("A"));
    CHECK(detail["properties"].is_array());
    bool sawPosition = false;
    bool sawRotation = false;
    for (std::size_t i = 0; i < detail["properties"].size(); ++i)
    {
        const ct::Json& property = detail["properties"][i];
        if (property["name"].as_cstr("") == ct::String("position"))
        {
            sawPosition = true;
            CHECK(property["type"].as_cstr("") == ct::String("Vec3"));
            CHECK(property["value"].size() == 3);
        }
        if (property["name"].as_cstr("") == ct::String("rotation"))
        {
            sawRotation = true;
            CHECK(property["unit"].as_cstr("") == ct::String("degrees"));
        }
    }
    CHECK(sawPosition && sawRotation);
    node["path"] = ".";
    CHECK(dataOf(client.tool("get_node", node))["parent"].is_null());
    node["path"] = "/A";
    CHECK(dataOf(client.tool("get_node", node))["name"].as_cstr("") == ct::String("A"));
    node["path"] = "Nothing/Here";
    const ct::Json missing = client.tool("get_node", node);
    CHECK(isError(missing));
    CHECK(textOf(missing).contains("No node at the path"));
    const ct::Json noPath = client.tool("get_node", args());
    CHECK(isError(noPath));
    CHECK(textOf(noPath).contains("Missing required argument 'path'"));
    ct::Json typo = args();
    typo["pth"] = "A";
    const ct::Json typoResult = client.tool("get_node", typo);
    CHECK(isError(typoResult));
    CHECK(textOf(typoResult).contains("Unknown argument 'pth'"));
    CHECK(textOf(typoResult).contains("path"));
    typo = args();
    typo["path"] = 5;
    CHECK(textOf(client.tool("get_node", typo)).contains("must be of type string"));
    typo["path"] = ct::Json();
    CHECK(textOf(client.tool("get_node", typo)).contains("Missing required argument"));
    ct::Json depth = args();
    depth["max_depth"] = 1.5;
    CHECK(textOf(client.tool("get_scene_tree", depth)).contains("must be of type integer"));
    ct::Json view = args();
    view["view"] = "minimap";
    CHECK(textOf(client.tool("screenshot", view)).contains("must be one of: scene, game"));
    ct::Json onlyNull = args();
    onlyNull["path"] = ct::Json();
    CHECK(succeeded(client.tool("get_scene_tree", onlyNull)));
    CHECK(textOf(client.tool("get_selection", typo)).contains("takes no arguments"));

    const ct::Json classes = dataOf(client.tool("list_classes"));
    CHECK(classes["classes"].size() > 10);
    CHECK(
            [&]
            {
                for (std::size_t i = 0; i < classes["classes"].size(); ++i)
                    if (classes["classes"][i]["name"].as_cstr("") == ct::String("OmniLight3D"))
                        return true;
                return false;
            }());
    ct::Json one = args();
    one["class"] = "OmniLight3D";
    const ct::Json omni = dataOf(client.tool("list_classes", one));
    CHECK(omni["parent"].as_cstr("") == ct::String("Light3D"));
    CHECK(omni["creatable"].as_bool(false));
    CHECK(omni["properties"].size() > 5);
    CHECK(
            [&]
            {
                for (std::size_t i = 0; i < omni["properties"].size(); ++i)
                    if (omni["properties"][i]["name"].as_cstr("") == ct::String("omni_range") &&
                            omni["properties"][i]["hint"].as_cstr("") == ct::String("range"))
                        return true;
                return false;
            }());
    one["class"] = "NoSuchClass";
    CHECK(isError(client.tool("list_classes", one)));
    ct::Json filter = args();
    filter["filter"] = "light";
    const ct::Json lights = dataOf(client.tool("list_classes", filter));
    CHECK(lights["classes"].size() >= 3 && lights["classes"].size() < classes["classes"].size());

    const ct::Json assets = dataOf(client.tool("list_assets"));
    CHECK(assets["folder"].as_cstr("") == ct::String("Assets"));
    CHECK(assets["entries"].is_array());
    ct::Json folder = args();
    folder["folder"] = "../..";
    CHECK(isError(client.tool("list_assets", folder)));
    folder["folder"] = "Nope";
    CHECK(isError(client.tool("list_assets", folder)));
    folder["folder"] = "";
    CHECK(succeeded(client.tool("list_assets", folder)));

    rig.selection.clear();
    rig.selection.add(b);
    CHECK(dataOf(client.tool("get_selection"))["selection"][0]["path"].as_cstr("") ==
            ct::String("A/B"));
    rig.selection.clear();
    CHECK(dataOf(client.tool("get_selection"))["selection"].size() == 0);

    const ct::Json settings = dataOf(client.tool("get_project_settings"));
    CHECK(settings["project"]["name"].as_cstr("") == ct::String("Mcp"));
    CHECK(settings["project"]["asset_root"].as_cstr("") == ct::String("Assets"));
    CHECK(settings["settings"].is_array());
    ct::Json all = args();
    all["include_defaults"] = true;
    all["filter"] = "display/";
    const ct::Json displays = dataOf(client.tool("get_project_settings", all));
    CHECK(displays["settings"].size() > 0);
    CHECK(ct::String(displays["settings"][0]["name"].as_cstr("")).size() > 0);

    ct::Json lines = args();
    lines["lines"] = 5;
    CHECK(dataOf(client.tool("get_console", lines))["lines"].is_array());
}

void testSceneEdits(Rig& rig, Client& client)
{
    rig.fresh();
    CHECK(!rig.document.dirty());

    ct::Json create = args();
    create["class"] = "Node3D";
    create["name"] = "Rig";
    ct::Json properties = args();
    properties["position"] = vec(1, 2, 3);
    properties["rotation"] = vec(0, 90, 0);
    properties["scale"] = vec(2, 2, 2);
    create["properties"] = properties;
    const ct::Json created = client.tool("create_node", create);
    CHECK(succeeded(created));
    const ct::Json createdData = dataOf(created);
    CHECK(createdData["node"]["path"].as_cstr("") == ct::String("Rig"));
    CHECK(createdData["node"]["class"].as_cstr("") == ct::String("Node3D"));
    Node3D* rigNode = nodeAt(rig, "Rig") ? nodeAt(rig, "Rig")->castTo<Node3D>() : nullptr;
    CHECK(rigNode != nullptr);
    if (!rigNode) return;
    CHECK(nearly(rigNode->getPosition().y, 2.0));
    CHECK(nearly(rigNode->getRotationDegrees().y, 90.0, 0.01));
    CHECK(nearly(rigNode->getScale().x, 2.0));
    CHECK(rig.document.dirty());
    CHECK(rig.selection.isSelected(rigNode));
    CHECK(rig.history().currentActionName() == "MCP: Create Node");
    CHECK(rig.history().undoCount() >= 1);

    ct::Json bad = args();
    bad["class"] = "NoSuchNode";
    CHECK(isError(client.tool("create_node", bad)));
    bad["class"] = "Node3D";
    bad["parent_path"] = "Missing";
    CHECK(isError(client.tool("create_node", bad)));
    bad = args();
    bad["class"] = "Node3D";
    bad["name"] = "Bad/Name";
    const ct::Json badName = client.tool("create_node", bad);
    CHECK(isError(badName));
    CHECK(textOf(badName).contains("Invalid node name"));
    bad = args();
    bad["class"] = "Node3D";
    ct::Json wrong = args();
    wrong["position"] = "left";
    bad["properties"] = wrong;
    CHECK(isError(client.tool("create_node", bad)));
    wrong = args();
    wrong["not_a_property"] = 1;
    bad["properties"] = wrong;
    const ct::Json unknownProperty = client.tool("create_node", bad);
    CHECK(isError(unknownProperty));
    CHECK(textOf(unknownProperty).contains("not_a_property"));
    CHECK(rig.document.scene()->getChildCount() == 1);

    ct::Json child = args();
    child["class"] = "OmniLight3D";
    child["parent_path"] = "Rig";
    ct::Json lightProperties = args();
    lightProperties["light_energy"] = 3.5;
    lightProperties["light_color"] = "#ff8000";
    lightProperties["omni_range"] = 12;
    child["properties"] = lightProperties;
    CHECK(succeeded(client.tool("create_node", child)));
    OmniLight3D* light = nodeAt(rig, "Rig/OmniLight3D")
                                 ? nodeAt(rig, "Rig/OmniLight3D")->castTo<OmniLight3D>()
                                 : nullptr;
    CHECK(light != nullptr);
    if (light)
    {
        CHECK(nearly(light->getEnergy(), 3.5));
        CHECK(nearly(light->getColor().x, 1.0));
        CHECK(nearly(light->getColor().y, 128.0 / 255.0, 0.01));
    }

    ct::Json set = args();
    set["path"] = "Rig";
    ct::Json values = args();
    values["position"] = vec(0, 5, 0);
    values["rotation"] = vec(0, 0, 45);
    values["visible"] = false;
    set["properties"] = values;
    const ct::Json changed = client.tool("set_properties", set);
    CHECK(succeeded(changed));
    CHECK(nearly(dataOf(changed)["properties"]["position"][1].as_double(), 5.0));
    CHECK(nearly(dataOf(changed)["properties"]["rotation"][2].as_double(), 45.0, 0.01));
    CHECK(!rigNode->isVisible());
    CHECK(nearly(rigNode->getRotationDegrees().z, 45.0, 0.01));
    CHECK(rig.history().currentActionName() == "MCP: Set Properties");
    const std::size_t undoCount = rig.history().undoCount();
    values = args();
    values["position"] = vec(7, 7, 7);
    values["visible"] = true;
    set["properties"] = values;
    CHECK(succeeded(client.tool("set_properties", set)));
    CHECK(rig.history().undoCount() == undoCount + 1);
    CHECK(rigNode->isVisible());
    rig.undoKey();
    CHECK(!rigNode->isVisible());
    CHECK(nearly(rigNode->getPosition().x, 0.0));
    CHECK(nearly(rigNode->getPosition().y, 5.0));
    rig.redoKey();
    CHECK(rigNode->isVisible());
    CHECK(nearly(rigNode->getPosition().x, 7.0));

    values = args();
    values["position"] = vec(0, 0, 0);
    values["bogus"] = 3;
    set["properties"] = values;
    CHECK(isError(client.tool("set_properties", set)));
    CHECK(nearly(rigNode->getPosition().x, 7.0));
    values = args();
    values["name"] = "Other";
    set["properties"] = values;
    CHECK(textOf(client.tool("set_properties", set)).contains("rename_node"));
    set["properties"] = args();
    CHECK(isError(client.tool("set_properties", set)));
    set["path"] = "Missing";
    values = args();
    values["visible"] = true;
    set["properties"] = values;
    CHECK(isError(client.tool("set_properties", set)));

    ct::Json rename = args();
    rename["path"] = "Rig";
    rename["name"] = "Pivot";
    CHECK(succeeded(client.tool("rename_node", rename)));
    CHECK(rigNode->getName() == "Pivot");
    CHECK(rig.history().currentActionName() == "MCP: Rename Node");
    rename["name"] = "Bad:Name";
    CHECK(isError(client.tool("rename_node", rename)));
    {
        ct::Json twin = args();
        twin["class"] = "Node3D";
        twin["name"] = "Twin";
        CHECK(succeeded(client.tool("create_node", twin)));
        twin["name"] = "Other";
        CHECK(succeeded(client.tool("create_node", twin)));
        ct::Json clash = args();
        clash["path"] = "Other";
        clash["name"] = "Twin";
        const ct::Json clashed = client.tool("rename_node", clash);
        CHECK(succeeded(clashed));
        CHECK(!(dataOf(clashed)["node"]["name"].as_cstr("") == ct::String("Twin")));
        CHECK(nodeAt(rig, "Twin") != nullptr);
        ct::Json removal = args();
        removal["path"] = dataOf(clashed)["node"]["path"];
        CHECK(succeeded(client.tool("delete_node", removal)));
        removal["path"] = "Twin";
        CHECK(succeeded(client.tool("delete_node", removal)));
    }

    ct::Json second = args();
    second["class"] = "Node3D";
    second["name"] = "Holder";
    CHECK(succeeded(client.tool("create_node", second)));
    ct::Json move = args();
    move["path"] = "Pivot";
    move["new_parent_path"] = "Holder";
    CHECK(succeeded(client.tool("reparent_node", move)));
    CHECK(nodeAt(rig, "Holder/Pivot") == rigNode);
    CHECK(rig.history().currentActionName() == "MCP: Reparent Node");
    move["new_parent_path"] = "Holder/Pivot";
    move["path"] = "Holder";
    CHECK(isError(client.tool("reparent_node", move)));
    move["path"] = ".";
    move["new_parent_path"] = "Holder";
    const ct::Json rootMove = client.tool("reparent_node", move);
    CHECK(isError(rootMove));
    CHECK(textOf(rootMove).contains("tree root"));

    ct::Json duplicate = args();
    duplicate["path"] = "Holder";
    const ct::Json copy = client.tool("duplicate_node", duplicate);
    CHECK(succeeded(copy));
    CHECK(nodeAt(rig, "Holder_0") != nullptr || nodeAt(rig, "Holder2") != nullptr ||
            rig.document.scene()->getChildCount() == 2);
    CHECK(rig.history().currentActionName() == "MCP: Duplicate Node(s)");
    CHECK(isError(client.tool("duplicate_node",
            []
            {
                ct::Json a = ct::Json::object();
                a["path"] = ".";
                return a;
            }())));

    ct::Json remove = args();
    remove["path"] = "Holder/Pivot/OmniLight3D";
    CHECK(succeeded(client.tool("delete_node", remove)));
    CHECK(nodeAt(rig, "Holder/Pivot/OmniLight3D") == nullptr);
    CHECK(rig.history().currentActionName() == "MCP: Delete Node(s)");
    rig.undoKey();
    CHECK(nodeAt(rig, "Holder/Pivot/OmniLight3D") != nullptr);
    remove["path"] = ".";
    const ct::Json rootDelete = client.tool("delete_node", remove);
    CHECK(isError(rootDelete));
    CHECK(textOf(rootDelete).contains("tree root"));
    CHECK(rig.document.scene() != nullptr);
    remove["path"] = "Nope";
    CHECK(isError(client.tool("delete_node", remove)));

    ct::Json select = args();
    select["path"] = "Holder";
    CHECK(succeeded(client.tool("select_node", select)));
    CHECK(rig.selection.selectedNodes().size() == 1);
    CHECK(rig.selection.isSelected(nodeAt(rig, "Holder")));
    ct::Json several = args();
    ct::Json paths = ct::Json::array();
    paths.push_back(ct::Json("Holder"));
    paths.push_back(ct::Json("Holder/Pivot"));
    several["paths"] = paths;
    CHECK(succeeded(client.tool("select_node", several)));
    CHECK(rig.selection.selectedNodes().size() == 2);
    several["paths"] = ct::Json::array();
    CHECK(succeeded(client.tool("select_node", several)));
    CHECK(rig.selection.selectedNodes().empty());
    select["path"] = "Missing";
    CHECK(isError(client.tool("select_node", select)));
}

void testPrimitivesAndMaterials(Rig& rig, Client& client)
{
    rig.fresh();
    ct::Json box = args();
    box["kind"] = "box";
    box["name"] = "Crate";
    ct::Json parameters = args();
    parameters["size"] = vec(2, 1, 3);
    box["parameters"] = parameters;
    box["position"] = vec(0, 0.5, 0);
    const ct::Json created = client.tool("create_primitive", box);
    CHECK(succeeded(created));
    MeshInstance3D* crate =
            nodeAt(rig, "Crate") ? nodeAt(rig, "Crate")->castTo<MeshInstance3D>() : nullptr;
    CHECK(crate != nullptr);
    if (!crate) return;
    CHECK(nearly(crate->getPosition().y, 0.5));
    CHECK(rig.history().currentActionName() == "MCP: Create Primitive");
    CHECK(dataOf(created)["mesh"].as_cstr("") == ct::String("primitive:box?size=2,1,3"));
    const MeshInfo* mesh = MeshStore::singleton()->get(crate->getMesh());
    CHECK(mesh != nullptr && mesh->surfaces.size() == 1);

    const char* const kinds[] = { "plane", "sphere", "cylinder", "cone", "capsule", "torus" };
    for (const char* kind: kinds)
    {
        ct::Json other = args();
        other["kind"] = kind;
        CHECK(succeeded(client.tool("create_primitive", other)));
    }
    CHECK(rig.document.scene()->getChildCount() == 7);
    CHECK(nodeAt(rig, "Plane") != nullptr);
    CHECK(nodeAt(rig, "Sphere") != nullptr);

    ct::Json sphere = args();
    sphere["kind"] = "sphere";
    ct::Json sphereParameters = args();
    sphereParameters["radius"] = 2.0;
    sphereParameters["slices"] = 12;
    sphere["parameters"] = sphereParameters;
    sphere["name"] = "Ball";
    CHECK(succeeded(client.tool("create_primitive", sphere)));
    sphereParameters["slices"] = 2;
    sphere["parameters"] = sphereParameters;
    CHECK(isError(client.tool("create_primitive", sphere)));
    sphereParameters = args();
    sphereParameters["radius"] = -1;
    sphere["parameters"] = sphereParameters;
    CHECK(isError(client.tool("create_primitive", sphere)));
    sphereParameters = args();
    sphereParameters["sides"] = 4;
    sphere["parameters"] = sphereParameters;
    CHECK(textOf(client.tool("create_primitive", sphere)).contains("sides"));
    ct::Json unknownKind = args();
    unknownKind["kind"] = "pyramid";
    CHECK(isError(client.tool("create_primitive", unknownKind)));
    CHECK(isError(client.tool("create_primitive", args())));
    {
        ct::Json outside = args();
        outside["path"] = "Crate";
        ct::Json meshes = args();
        meshes["mesh"] = "/etc/hostname.obj";
        outside["properties"] = meshes;
        const ct::Json absolute = client.tool("set_properties", outside);
        CHECK(isError(absolute));
        CHECK(textOf(absolute).contains("relative to the project"));
        meshes["mesh"] = "../../other/model.obj";
        outside["properties"] = meshes;
        CHECK(isError(client.tool("set_properties", outside)));
        meshes["mesh"] = "C:\\models\\a.obj";
        outside["properties"] = meshes;
        CHECK(isError(client.tool("set_properties", outside)));
        ct::Json texture = args();
        texture["name"] = "Leak";
        ct::Json leak = args();
        leak["albedo_texture"] = "/tmp/anything.png";
        texture["parameters"] = leak;
        CHECK(isError(client.tool("create_material", texture)));
        CHECK(!file_exists(
                rig.document.project().absolutePath("Assets/Materials/Leak.mat").c_str()));
    }

    ct::Json material = args();
    material["name"] = "Copper";
    ct::Json settings = args();
    settings["albedo_color"] = "#b87333";
    settings["metallic"] = 0.9;
    settings["roughness"] = 0.35;
    settings["emission_enabled"] = true;
    settings["emission"] = vec(0.1, 0.05, 0.0);
    settings["cull_mode"] = "Disabled";
    material["parameters"] = settings;
    const ct::Json madeMaterial = client.tool("create_material", material);
    CHECK(succeeded(madeMaterial));
    const ct::Json materialData = dataOf(madeMaterial);
    CHECK(materialData["file"].as_cstr("") == ct::String("Assets/Materials/Copper.mat"));
    CHECK(materialData["reference"].as_cstr("") ==
            ct::String("Assets/Materials/Copper.mat::Copper"));
    CHECK(materialData["applied"].size() == 6);
    CHECK(file_exists(rig.document.project().absolutePath("Assets/Materials/Copper.mat").c_str()));
    {
        MaterialLibraryData data;
        ct::String error;
        CHECK(MaterialLibraryFormat::load(
                rig.document.project().absolutePath("Assets/Materials/Copper.mat"), data, error));
        CHECK(data.materials.size() == 1);
        if (data.materials.size() == 1)
        {
            const MaterialInfo& info = data.materials[0].info;
            CHECK(nearly(info.metallic, 0.9));
            CHECK(nearly(info.roughness, 0.35));
            CHECK(nearly(info.albedo.x, 184.0 / 255.0, 0.01));
            CHECK(info.cull == MaterialCull::Disabled);
            CHECK(nearly(info.emission.x, 0.1, 0.01));
        }
    }
    CHECK(isError(client.tool("create_material", material)));
    material["name"] = "Bad";
    settings = args();
    settings["not_a_field"] = 1;
    material["parameters"] = settings;
    CHECK(isError(client.tool("create_material", material)));
    CHECK(!file_exists(rig.document.project().absolutePath("Assets/Materials/Bad.mat").c_str()));
    settings = args();
    settings["metallic"] = "shiny";
    material["parameters"] = settings;
    CHECK(isError(client.tool("create_material", material)));
    material["name"] = "../Evil";
    material["parameters"] = args();
    CHECK(isError(client.tool("create_material", material)));
    CHECK(isError(client.tool("create_material", args())));

    ct::Json assign = args();
    assign["path"] = "Crate";
    assign["material"] = "Assets/Materials/Copper.mat";
    const ct::Json assigned = client.tool("assign_material", assign);
    CHECK(succeeded(assigned));
    CHECK(crate->getSurfaceMaterial(0).valid());
    CHECK(rig.history().currentActionName() == "MCP: Assign Material");
    rig.undoKey();
    CHECK(!crate->getSurfaceMaterial(0).valid());
    rig.redoKey();
    CHECK(crate->getSurfaceMaterial(0).valid());
    const MaterialInfo* assignedInfo =
            Main::rendering()->materials().get(crate->getSurfaceMaterial(0));
    CHECK(assignedInfo != nullptr && assignedInfo->name == "Copper");
    assign["material"] = "Assets/Materials/Copper.mat::Copper";
    assign["override"] = true;
    CHECK(succeeded(client.tool("assign_material", assign)));
    CHECK(crate->getMaterialOverride().valid());
    assign["override"] = false;
    assign["surface"] = 5;
    CHECK(isError(client.tool("assign_material", assign)));
    assign["surface"] = 0;
    assign["material"] = "Assets/Materials/Missing.mat";
    CHECK(isError(client.tool("assign_material", assign)));
    assign["material"] = "../outside.mat";
    CHECK(isError(client.tool("assign_material", assign)));
    assign["material"] = "Assets/Materials/Copper.mat";
    assign["path"] = ".";
    const ct::Json notMesh = client.tool("assign_material", assign);
    CHECK(isError(notMesh));
    CHECK(textOf(notMesh).contains("MeshInstance3D"));
    assign["path"] = "Nope";
    CHECK(isError(client.tool("assign_material", assign)));

    ct::Json folder = args();
    folder["folder"] = "Assets/Materials";
    const ct::Json listing = dataOf(client.tool("list_assets", folder));
    CHECK(listing["entries"].size() >= 1);
    CHECK(listing["entries"][0]["kind"].as_cstr("") == ct::String("Material"));
    CHECK(listing["entries"][0]["path"].as_cstr("") == ct::String("Assets/Materials/Copper.mat"));
}

void testImport(Rig& rig, Client& client)
{
    const ct::String source = testRoot() + "/outside";
    CHECK(fs_create_directory(source.c_str()));
    const ct::String image = source + "/stripes.png";
    {
        Pixmap pixels(8, 8, 4);
        pixels.fill(200, 40, 40, 255);
        CHECK(pixels.save(image.c_str()));
    }
    CHECK(file_write_text((source + "/notes.txt").c_str(), "text"));

    ct::Json import = args();
    import["source"] = image;
    import["folder"] = "Assets/Textures";
    const ct::Json done = client.tool("import_asset", import);
    CHECK(succeeded(done));
    CHECK(dataOf(done)["imported"][0].as_cstr("") == ct::String("Assets/Textures/stripes.png"));
    CHECK(file_exists(rig.document.project().absolutePath("Assets/Textures/stripes.png").c_str()));
    ct::Json folder = args();
    folder["folder"] = "Assets/Textures";
    CHECK(dataOf(client.tool("list_assets", folder))["entries"][0]["kind"].as_cstr("") ==
            ct::String("Texture"));

    const ct::Json clash = client.tool("import_asset", import);
    CHECK(isError(clash));
    CHECK(textOf(clash).contains("overwrite"));
    import["overwrite"] = true;
    CHECK(succeeded(client.tool("import_asset", import)));

    ct::Json text = args();
    text["source"] = source + "/notes.txt";
    CHECK(isError(client.tool("import_asset", text)));
    text["source"] = source + "/absent.png";
    CHECK(isError(client.tool("import_asset", text)));
    text["source"] = image;
    text["folder"] = "../escape";
    CHECK(isError(client.tool("import_asset", text)));
    CHECK(isError(client.tool("import_asset", args())));

    ct::Json material = args();
    material["name"] = "Striped";
    ct::Json settings = args();
    settings["albedo_texture"] = "Assets/Textures/stripes.png";
    material["parameters"] = settings;
    const ct::Json textured = client.tool("create_material", material);
    CHECK(succeeded(textured));
    CHECK(dataOf(textured)["applied"].size() == 1);
}

void testEnvironment(Rig& rig, Client& client)
{
    rig.fresh();
    ct::Json environment = args();
    ct::Json properties = args();
    properties["background_mode"] = "Custom Color";
    properties["background_color"] = color(0.5, 0.6, 0.7);
    properties["fog_enabled"] = true;
    properties["fog_light_color"] = "#aabbcc";
    properties["fog_density"] = 0.03;
    properties["tonemap_mode"] = 3;
    environment["properties"] = properties;
    const ct::Json created = client.tool("create_environment", environment);
    CHECK(succeeded(created));
    WorldEnvironment* node = nodeAt(rig, "WorldEnvironment")
                                     ? nodeAt(rig, "WorldEnvironment")->castTo<WorldEnvironment>()
                                     : nullptr;
    CHECK(node != nullptr);
    if (node)
    {
        PropertyValue value;
        CHECK(NodeClassDB::getProperty(node, "fog_enabled", value) && value.asBool());
        CHECK(NodeClassDB::getProperty(node, "fog_density", value) &&
                nearly(value.asFloat(), 0.03, 1e-4));
        CHECK(NodeClassDB::getProperty(node, "background_mode", value) && value.asInt() == 1);
        CHECK(NodeClassDB::getProperty(node, "fog_light_color", value) &&
                nearly(value.asColor().z, 204.0 / 255.0, 0.01));
    }
    CHECK(rig.history().currentActionName() == "MCP: Create Environment");
    const ct::Json again = client.tool("create_environment", args());
    CHECK(isError(again));
    CHECK(textOf(again).contains("already has a WorldEnvironment"));
    rig.undoKey();
    CHECK(nodeAt(rig, "WorldEnvironment") == nullptr);
    properties = args();
    properties["fog_color_nope"] = 1;
    environment["properties"] = properties;
    CHECK(isError(client.tool("create_environment", environment)));
    CHECK(nodeAt(rig, "WorldEnvironment") == nullptr);
}

void testScenes(Rig& rig, Client& client)
{
    rig.fresh();
    ct::Json create = args();
    create["class"] = "Node3D";
    create["name"] = "Keep";
    CHECK(succeeded(client.tool("create_node", create)));
    CHECK(rig.document.dirty());

    const ct::Json refused = client.tool("new_scene");
    CHECK(isError(refused));
    CHECK(textOf(refused).contains("unsaved"));
    CHECK(nodeAt(rig, "Keep") != nullptr);
    ct::Json open = args();
    open["path"] = "Scenes/other.scene.json";
    CHECK(isError(client.tool("open_scene", open)));
    CHECK(nodeAt(rig, "Keep") != nullptr);

    const ct::Json noName = client.tool("save_scene");
    CHECK(isError(noName));
    CHECK(textOf(noName).contains("name"));
    ct::Json save = args();
    save["name"] = "bad/name";
    CHECK(isError(client.tool("save_scene", save)));
    save["name"] = "first";
    const ct::Json saved = client.tool("save_scene", save);
    CHECK(succeeded(saved));
    CHECK(!rig.document.dirty());
    CHECK(dataOf(saved)["scene"]["file"].as_cstr("") == ct::String("Scenes/first.scene.json"));
    CHECK(file_exists(rig.document.project().absolutePath("Scenes/first.scene.json").c_str()));
    create["name"] = "Second";
    CHECK(succeeded(client.tool("create_node", create)));
    CHECK(rig.document.dirty());
    CHECK(succeeded(client.tool("save_scene")));
    CHECK(!rig.document.dirty());

    CHECK(succeeded(client.tool("new_scene")));
    CHECK(rig.document.scene()->getChildCount() == 0);
    open["path"] = "first";
    CHECK(succeeded(client.tool("open_scene", open)));
    CHECK(nodeAt(rig, "Keep") != nullptr);
    CHECK(nodeAt(rig, "Second") != nullptr);
    open["path"] = "Scenes/first.scene.json";
    CHECK(succeeded(client.tool("open_scene", open)));
    open["path"] = "missing";
    CHECK(isError(client.tool("open_scene", open)));
    open["path"] = "../x.scene.json";
    CHECK(isError(client.tool("open_scene", open)));
    CHECK(isError(client.tool("open_scene", args())));

    create["name"] = "Third";
    CHECK(succeeded(client.tool("create_node", create)));
    ct::Json discard = args();
    discard["discard_changes"] = true;
    CHECK(succeeded(client.tool("new_scene", discard)));
    CHECK(nodeAt(rig, "Third") == nullptr);
    CHECK(rig.history().undoCount() == 0);
}

void testCode(Rig& rig, Client& client)
{
    rig.fresh();
    ct::Json box = args();
    box["kind"] = "box";
    box["name"] = "Spinner";
    CHECK(succeeded(client.tool("create_primitive", box)));

    ct::Json script = args();
    script["path"] = "spin.py";
    script["source"] =
            "class Spin(MeshInstance3D):\n    speed = 120.0\n\n    def _process(self, delta):\n"
            "        self.rotation_degrees.y += self.speed * delta\n";
    const ct::Json written = client.tool("write_script", script);
    CHECK(succeeded(written));
    CHECK(dataOf(written)["path"].as_cstr("") == ct::String("Assets/Scripts/spin.py"));
    CHECK(!dataOf(written)["replaced"].as_bool(true));
    CHECK(file_exists(rig.document.project().absolutePath("Assets/Scripts/spin.py").c_str()));
    CHECK(dataOf(client.tool("write_script", script))["replaced"].as_bool(false));

    ct::Json outside = args();
    outside["path"] = "Assets/Other/spin.py";
    outside["source"] = "x = 1\n";
    CHECK(isError(client.tool("write_script", outside)));
    outside["path"] = "Assets/Scripts/../../evil.py";
    CHECK(isError(client.tool("write_script", outside)));
    outside["path"] = "Assets/Scripts/data.txt";
    CHECK(isError(client.tool("write_script", outside)));
    outside["path"] = "/etc/evil.py";
    CHECK(isError(client.tool("write_script", outside)));
    CHECK(isError(client.tool("write_script", args())));

    ct::Json attach = args();
    attach["node_path"] = "Spinner";
    attach["script_path"] = "Assets/Scripts/spin.py";
    CHECK(succeeded(client.tool("attach_script", attach)));
    Node* spinner = nodeAt(rig, "Spinner");
    CHECK(spinner != nullptr && spinner->getScript() == "Assets/Scripts/spin.py");
    CHECK(rig.history().currentActionName() == "MCP: Attach Script");
    rig.undoKey();
    CHECK(spinner->getScript().empty());
    rig.redoKey();
    CHECK(spinner->getScript() == "Assets/Scripts/spin.py");
    {
        ct::Json second = args();
        second["class"] = "MeshInstance3D";
        second["name"] = "Second";
        ct::Json values = args();
        values["script"] = "Assets/Scripts/spin.py";
        second["properties"] = values;
        CHECK(succeeded(client.tool("create_node", second)));
        CHECK(nodeAt(rig, "Second") &&
                nodeAt(rig, "Second")->getScript() == "Assets/Scripts/spin.py");
    }
    attach["script_path"] = "Assets/Scripts/missing.py";
    CHECK(isError(client.tool("attach_script", attach)));
    attach["script_path"] = "Assets/Scripts/spin.py";
    attach["node_path"] = "Nobody";
    CHECK(isError(client.tool("attach_script", attach)));

    const ct::Json started = client.tool("play");
    CHECK(succeeded(started));
    CHECK(rig.play.playing());
    CHECK(dataOf(started)["playing"].as_bool(false));
    rig.frames(30);
    Node* playing = nodeAt(rig, "Spinner");
    CHECK(playing != nullptr);
    if (playing)
    {
        const Node3D* spatial = playing->castTo<Node3D>();
        CHECK(spatial != nullptr && spatial->getRotationDegrees().y != 0.0f);
    }
    ct::Json create = args();
    create["class"] = "Node3D";
    const ct::Json blocked = client.tool("create_node", create);
    CHECK(isError(blocked));
    CHECK(textOf(blocked).contains("stop"));
    CHECK(isError(client.tool("set_properties", args())));
    CHECK(isError(client.tool("new_scene")));
    ct::Json lines = args();
    lines["lines"] = 100;
    CHECK(dataOf(client.tool("get_console", lines))["lines"].is_array());
    const ct::Json stopped = client.tool("stop");
    CHECK(succeeded(stopped));
    CHECK(!rig.play.playing());
    rig.frames(3);
    Node* back = nodeAt(rig, "Spinner");
    CHECK(back != nullptr);
    if (back) CHECK(nearly(back->castTo<Node3D>()->getRotationDegrees().y, 0.0, 0.01));
    CHECK(isError(client.tool("stop")));
    CHECK(succeeded(client.tool("create_node", create)));
}

void testScreenshots(Rig& rig, Client& client)
{
    rig.fresh();
    ct::Json box = args();
    box["kind"] = "box";
    box["position"] = vec(0, 0, -4);
    CHECK(succeeded(client.tool("create_primitive", box)));
    ct::Json light = args();
    light["class"] = "DirectionalLight3D";
    ct::Json rotation = args();
    rotation["rotation"] = vec(-40, 30, 0);
    light["properties"] = rotation;
    CHECK(succeeded(client.tool("create_node", light)));
    rig.shell.showPanel(EditorShell::kScenePanel);
    rig.frames(6);

    const ct::Json shot = client.tool("screenshot");
    CHECK(succeeded(shot));
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> png;
    CHECK(decodePng(shot, width, height, png));
    CHECK(width <= 1024 && width >= 64 && height >= 32);
    CHECK(imageHasDetail(png));
    CHECK(textOf(shot).contains("scene view"));

    ct::Json small = args();
    small["max_width"] = 96;
    const ct::Json thumb = client.tool("screenshot", small);
    int smallWidth = 0;
    int smallHeight = 0;
    ct::Vector<std::uint8_t> smallPng;
    CHECK(decodePng(thumb, smallWidth, smallHeight, smallPng));
    CHECK(smallWidth == 96);

    ct::Json game = args();
    game["view"] = "game";
    const ct::Json hidden = client.tool("screenshot", game);
    CHECK(isError(hidden));
    CHECK(textOf(hidden).contains("Game"));
    rig.frames(6);
    const ct::Json noCamera = client.tool("screenshot", game);
    CHECK(isError(noCamera));
    CHECK(textOf(noCamera).contains("camera"));

    ct::Json camera = args();
    camera["class"] = "Camera3D";
    ct::Json cameraProperties = args();
    cameraProperties["position"] = vec(0, 1, 3);
    cameraProperties["current"] = true;
    camera["properties"] = cameraProperties;
    CHECK(succeeded(client.tool("create_node", camera)));
    rig.frames(8);
    const ct::Json gameShot = client.tool("screenshot", game);
    CHECK(succeeded(gameShot));
    CHECK(decodePng(gameShot, width, height, png));
    CHECK(imageHasDetail(png));
    CHECK(textOf(gameShot).contains("game view"));

    ct::Json wrong = args();
    wrong["view"] = "minimap";
    CHECK(isError(client.tool("screenshot", wrong)));
    rig.shell.showPanel(EditorShell::kScenePanel);
    rig.frames(6);
}

void testEndToEnd(Rig& rig, Client& client)
{
    CHECK(succeeded(client.tool("new_scene",
            []
            {
                ct::Json a = ct::Json::object();
                a["discard_changes"] = true;
                return a;
            }())));
    rig.shell.showPanel(EditorShell::kScenePanel);

    ct::Json floor = args();
    floor["kind"] = "plane";
    floor["name"] = "Floor";
    ct::Json floorParameters = args();
    floorParameters["width"] = 20;
    floorParameters["depth"] = 20;
    floor["parameters"] = floorParameters;
    CHECK(succeeded(client.tool("create_primitive", floor)));
    ct::Json ground = args();
    ground["name"] = "Ground";
    ct::Json groundParameters = args();
    groundParameters["albedo_color"] = color(0.2, 0.45, 0.2);
    groundParameters["roughness"] = 0.9;
    ground["parameters"] = groundParameters;
    CHECK(succeeded(client.tool("create_material", ground)));
    ct::Json assignFloor = args();
    assignFloor["path"] = "Floor";
    assignFloor["material"] = "Assets/Materials/Ground.mat";
    CHECK(succeeded(client.tool("assign_material", assignFloor)));

    ct::Json box = args();
    box["kind"] = "box";
    box["name"] = "Box";
    box["position"] = vec(0, 0.5, 0);
    CHECK(succeeded(client.tool("create_primitive", box)));
    ct::Json orange = args();
    orange["name"] = "Orange";
    ct::Json orangeParameters = args();
    orangeParameters["albedo_color"] = "#f27319";
    orangeParameters["roughness"] = 0.5;
    orange["parameters"] = orangeParameters;
    CHECK(succeeded(client.tool("create_material", orange)));
    ct::Json assignBox = args();
    assignBox["path"] = "Box";
    assignBox["material"] = "Assets/Materials/Orange.mat";
    CHECK(succeeded(client.tool("assign_material", assignBox)));

    ct::Json sun = args();
    sun["class"] = "DirectionalLight3D";
    sun["name"] = "Sun";
    ct::Json sunProperties = args();
    sunProperties["rotation"] = vec(-50, 30, 0);
    sunProperties["shadow_enabled"] = true;
    sun["properties"] = sunProperties;
    CHECK(succeeded(client.tool("create_node", sun)));

    ct::Json camera = args();
    camera["class"] = "Camera3D";
    camera["name"] = "MainCamera";
    ct::Json cameraProperties = args();
    cameraProperties["position"] = vec(0, 3, 6);
    cameraProperties["rotation"] = vec(-25, 0, 0);
    cameraProperties["current"] = true;
    camera["properties"] = cameraProperties;
    CHECK(succeeded(client.tool("create_node", camera)));

    ct::Json environment = args();
    ct::Json environmentProperties = args();
    environmentProperties["background_mode"] = "Custom Color";
    environmentProperties["background_color"] = "#87a8c8";
    environmentProperties["fog_enabled"] = true;
    environmentProperties["fog_light_color"] = "#b4c6d8";
    environmentProperties["fog_density"] = 0.02;
    environment["properties"] = environmentProperties;
    CHECK(succeeded(client.tool("create_environment", environment)));

    ct::Json script = args();
    script["path"] = "Assets/Scripts/turn.py";
    script["source"] =
            "class Turn(MeshInstance3D):\n    speed = 90.0\n\n    def _process(self, delta):\n"
            "        self.rotation_degrees.y += self.speed * delta\n";
    CHECK(succeeded(client.tool("write_script", script)));
    ct::Json attach = args();
    attach["node_path"] = "Box";
    attach["script_path"] = "Assets/Scripts/turn.py";
    CHECK(succeeded(client.tool("attach_script", attach)));

    ct::Json save = args();
    save["name"] = "built";
    const ct::Json saved = client.tool("save_scene", save);
    CHECK(succeeded(saved));
    CHECK(!rig.document.dirty());

    rig.frames(8);
    const ct::Json shot = client.tool("screenshot");
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> png;
    CHECK(decodePng(shot, width, height, png));
    CHECK(imageHasDetail(png));
    ct::Json gameView = args();
    gameView["view"] = "game";
    client.tool("screenshot", gameView);
    rig.frames(8);
    const ct::Json gameShot = client.tool("screenshot", gameView);
    CHECK(decodePng(gameShot, width, height, png));
    CHECK(imageHasDetail(png));
    CHECK(countPixels(png, &orangeish) > 40);
    CHECK(countPixels(png, &greenish) > 400);

    ct::Json discard = args();
    discard["discard_changes"] = true;
    CHECK(succeeded(client.tool("new_scene", discard)));
    CHECK(rig.document.scene()->getChildCount() == 0);
    ct::Json open = args();
    open["path"] = "built";
    CHECK(succeeded(client.tool("open_scene", open)));
    rig.frames(4);

    const ct::Json tree = dataOf(client.tool("get_scene_tree"));
    const ct::Json& children = tree["tree"]["children"];
    CHECK(children.size() == 5);
    CHECK(nodeAt(rig, "Floor") && nodeAt(rig, "Box") && nodeAt(rig, "Sun") &&
            nodeAt(rig, "MainCamera") && nodeAt(rig, "WorldEnvironment"));
    CHECK(nodeAt(rig, "Box") && nodeAt(rig, "Box")->getScript() == "Assets/Scripts/turn.py");
    if (MeshInstance3D* mesh =
                    nodeAt(rig, "Box") ? nodeAt(rig, "Box")->castTo<MeshInstance3D>() : nullptr)
    {
        const MaterialInfo* info = Main::rendering()->materials().get(mesh->getSurfaceMaterial(0));
        CHECK(info != nullptr && info->name == "Orange");
        if (info) CHECK(nearly(info->albedo.x, 242.0 / 255.0, 0.01));
    }
    if (WorldEnvironment* env =
                    nodeAt(rig, "WorldEnvironment")
                            ? nodeAt(rig, "WorldEnvironment")->castTo<WorldEnvironment>()
                            : nullptr)
    {
        PropertyValue value;
        CHECK(NodeClassDB::getProperty(env, "fog_enabled", value) && value.asBool());
        CHECK(NodeClassDB::getProperty(env, "fog_density", value) &&
                nearly(value.asFloat(), 0.02, 1e-4));
    }
    if (Camera3D* cam = nodeAt(rig, "MainCamera") ? nodeAt(rig, "MainCamera")->castTo<Camera3D>()
                                                  : nullptr)
        CHECK(cam->isCurrent());
    if (Node3D* light = nodeAt(rig, "Sun") ? nodeAt(rig, "Sun")->castTo<Node3D>() : nullptr)
        CHECK(nearly(light->getRotationDegrees().x, -50.0, 0.1));

    {
        ct::Json view = args();
        view["view"] = "game";
        rig.frames(10);
        const ct::Json again = client.tool("screenshot", view);
        ct::Vector<std::uint8_t> bytes;
        int w = 0;
        int h = 0;
        CHECK(decodePng(again, w, h, bytes));
        Pixmap image;
        CHECK(image.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
        CHECK(image.pixels != nullptr && abs(image.pixels[0] - 180) < 12 &&
                abs(image.pixels[1] - 198) < 12 && abs(image.pixels[2] - 216) < 12);
    }

    CHECK(succeeded(client.tool("play")));
    rig.frames(30);
    Node3D* box3d = nodeAt(rig, "Box") ? nodeAt(rig, "Box")->castTo<Node3D>() : nullptr;
    CHECK(box3d != nullptr && box3d->getRotationDegrees().y != 0.0f);
    CHECK(succeeded(client.tool("stop")));
    rig.frames(3);
}

void testUndoFromUi(Rig& rig, Client& client)
{
    rig.fresh();
    CHECK(!rig.history().hasUndo());
    ct::Json create = args();
    create["class"] = "Node3D";
    create["name"] = "Undoable";
    CHECK(succeeded(client.tool("create_node", create)));
    CHECK(nodeAt(rig, "Undoable") != nullptr);
    CHECK(rig.history().hasUndo());
    CHECK(rig.history().currentActionName() == "MCP: Create Node");
    CHECK(rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(nodeAt(rig, "Undoable") == nullptr);
    CHECK(!rig.document.dirty());
    CHECK(rig.history().redoActionName() == "MCP: Create Node");
    CHECK(rig.redoKey());
    CHECK(nodeAt(rig, "Undoable") != nullptr);
    CHECK(rig.document.dirty());

    EditorUndoRedoManager& undo = rig.history();
    CHECK(undo.namePrefix().empty());
}

void testPanel(Rig& rig)
{
    rig.shell.showPanel(EditorShell::kAiPanel);
    rig.frames(4);
    CHECK(rig.mcp.serverSwitch()->isVisible());

    const int blockedPort = findFreePort(rig.server);
    CHECK(blockedPort != 0);
    ct::TcpListener blocker;
    ct::Address address;
    ct::Address::parse("127.0.0.1", static_cast<std::uint16_t>(blockedPort), address);
    CHECK(blocker.bind(address));
    CHECK(blocker.listen());

    ct::String text;
    text.append_number(blockedPort);
    rig.mcp.portInput()->setText(text);
    rig.mcp.portInput()->committed.emit(text);
    CHECK(rig.settings.mcpPort == blockedPort);

    rt::Rect rect = rig.mcp.serverSwitch()->absoluteRect();
    rig.click(static_cast<int>(rect.x + 10), static_cast<int>(rect.y + rect.h * 0.5f));
    CHECK(!rig.server.running());
    CHECK(!rig.mcp.serverSwitch()->isOn());
    CHECK(rig.mcp.statusLabel()->text().starts_with("Error: Could not listen on 127.0.0.1:"));
    CHECK(rig.server.error().contains("Could not listen"));
    CHECK(rig.mcp.portInput()->isEnabled());
    blocker.close();

    rect = rig.mcp.serverSwitch()->absoluteRect();
    rig.click(static_cast<int>(rect.x + 10), static_cast<int>(rect.y + rect.h * 0.5f));
    CHECK(rig.server.running());
    CHECK(rig.mcp.serverSwitch()->isOn());
    CHECK(rig.server.port() == blockedPort);
    CHECK(rig.server.error().empty());
    CHECK(rig.mcp.statusLabel()->text().starts_with("Listening on 127.0.0.1:"));

    rig.mcp.portInput()->setText("9");
    rig.mcp.portInput()->committed.emit("9");
    CHECK(rig.mcp.portInput()->text() == text);
    CHECK(rig.settings.mcpPort == blockedPort);

    Client client(rig, blockedPort);
    CHECK(client.initialize());
    client.tool("get_scene_tree");
    client.tool("create_node", args());
    rig.frames(2);
    const ct::Vector<McpServer::LogEntry>& log = rig.server.log();
    CHECK(log.size() >= 4);
    CHECK(rig.mcp.logGrid()->cell(0, 1) == "create_node");
    CHECK(rig.mcp.logGrid()->cell(0, 3).starts_with("error: "));
    CHECK(rig.mcp.logGrid()->cell(1, 1) == "get_scene_tree");
    CHECK(rig.mcp.logGrid()->cell(1, 3) == "ok");
    CHECK(rig.mcp.logGrid()->cell(1, 0).size() == 8);
    CHECK(rig.mcp.logGrid()->cell(1, 2) == "{}");

    CHECK(rig.mcp.copyCommand());
    if (Device* device = Main::device())
    {
        if (device->clipboardHas())
        {
            ct::String expected = McpServer::addCommandFor(blockedPort);
            CHECK(ct::String(device->clipboardGet()) == expected);
        }
    }

    rig.mcp.clearButton()->clicked.emit();
    rig.frames(2);
    CHECK(rig.server.log().empty());
    CHECK(rig.mcp.logGrid()->cell(0, 1).empty());

    rect = rig.mcp.editsSwitch()->absoluteRect();
    rig.click(static_cast<int>(rect.x + 10), static_cast<int>(rect.y + rect.h * 0.5f));
    CHECK(rig.mcp.editsSwitch()->isOn());
    CHECK(rig.server.allowEdits());
    CHECK(rig.settings.mcpAllowEdits);
    CHECK(!rig.server.allowCode());
    rect = rig.mcp.codeSwitch()->absoluteRect();
    rig.click(static_cast<int>(rect.x + 10), static_cast<int>(rect.y + rect.h * 0.5f));
    CHECK(rig.server.allowCode());
    CHECK(rig.settings.mcpAllowCode);

    const ct::String iniPath = testRoot() + "/panel.ini";
    CHECK(rig.settings.save(iniPath));
    EditorSettings reloaded;
    CHECK(reloaded.load(iniPath));
    CHECK(reloaded.mcpPort == blockedPort);
    CHECK(reloaded.mcpAllowEdits);
    CHECK(reloaded.mcpAllowCode);
    McpServer another;
    CHECK(!another.running());
    {
        McpPanel anotherPanel(another, reloaded);
        CHECK(another.allowEdits() && another.allowCode());
        CHECK(!another.running());
        CHECK(another.port() == 0);
    }

    rig.click(static_cast<int>(rig.mcp.serverSwitch()->absoluteRect().x + 10),
            static_cast<int>(rig.mcp.serverSwitch()->absoluteRect().y + 8));
    CHECK(!rig.server.running());
    CHECK(rig.mcp.statusLabel()->text() == "Stopped");
    rig.mcp.editsSwitch()->setOn(false);
    rig.mcp.codeSwitch()->setOn(false);
    rig.shell.showPanel(EditorShell::kScenePanel);
    rig.frames(3);
}

void testLayout(Rig& rig)
{
    CHECK(EditorShell::panelName(EditorShell::kAiPanel) == ct::String("AI"));
    CHECK(rig.shell.panel(EditorShell::kAiPanel) != nullptr);
    const ct::String current = rig.shell.layout().c_str();
    CHECK(current.contains("AI"));
    CHECK(rig.shell.setLayout("H200[{Hierarchy}|H780[V450[{Scene,Game/0}|{Console,Assets/1}]|"
                              "{Inspector,Material/0}]]"));
    rig.frames(2);
    CHECK(ct::String(rig.shell.layout().c_str()).contains("AI"));
    rig.shell.showPanel(EditorShell::kAiPanel);
    rig.frames(3);
    CHECK(rig.mcp.serverSwitch()->isVisible());
    rig.shell.resetLayout();
    rig.frames(2);
    CHECK(ct::String(rig.shell.layout().c_str()).contains("AI"));
}

void testShutdown(Rig& rig)
{
    int port = 0;
    CHECK(startServer(rig, port));
    Client client(rig, port);
    CHECK(client.initialize());
    CHECK(rig.server.running());
    rig.server.stop();
    CHECK(!rig.server.running());
    const Reply closed = exchange(rig, port, "POST", "{}", client.headers());
    CHECK(!closed.sent);
    CHECK(rig.server.start(port));
    CHECK(rig.server.running());
    rig.server.stop();
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;
    if (argc > 2) gOnly = argv[2];

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor mcp test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;
    Engine::singleton()->setEditorHint(true);

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));
    CHECK(fs_create_directory((testRoot() + "/projects").c_str()));

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    Node3D* scene = tree.getRoot()->createChild<Node3D>();
    scene->setName("Scene");

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        CHECK(host.init());
        CHECK(shell.build());

        EditorSelection selection(tree);
        SceneTreePanel hierarchy(tree, selection);
        InspectorPanel inspector(tree, selection);
        CHECK(hierarchy.attach(shell.panel(0)));
        CHECK(inspector.attach(shell.panel(2)));
        hierarchy.setEditedScene(scene);

        EditorSettings settings;
        Project project;
        EditorDocument document(tree, selection, hierarchy, project, settings);
        document.setSettingsPath(testRoot() + "/settings.json");
        inspector.setChangeListener(&document);
        inspector.setProject(&project);
        hierarchy.setChangeListener(&document);
        ProjectUi ui(document);
        CHECK(ui.attach(shell.toolBar()));

        EditorViewportPanel panel(server);
        panel.bindEditor(tree, selection);
        panel.setChangeListener(&document);
        EditorGizmo* gizmo = panel.gizmo();
        CHECK(gizmo != nullptr);
        if (gizmo)
            gizmo->setRootProvider([&document]() -> Node* { return document.scene(); });
        CHECK(panel.attach(shell.panel(EditorShell::kScenePanel)));
        panel.setScenario(window->getScenario());
        panel.connectPreDraw();

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        actions.setViewport(&panel);

        GamePanel game(server, document);
        CHECK(game.attach(shell.panel(EditorShell::kGamePanel)));
        game.setScenario(window->getScenario());
        game.connectPreDraw();
        EditorPlay play(tree, document);
        CHECK(play.attach(shell.runBar()));
        play.setViewHandler(
                [&shell](bool showGame) {
                    shell.showPanel(showGame ? EditorShell::kGamePanel : EditorShell::kScenePanel);
                });

        EditorMenus menus(shell, ui, panel, actions, &play);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        MaterialPanel materials(server, tree, selection, document);
        CHECK(materials.attach(shell.panel(EditorShell::kMaterialPanel)));
        materials.bindViewport(panel);
        AssetsPanel assets(document, selection, hierarchy, actions, ui);
        CHECK(assets.attach(shell.panel(4)));
        assets.bindViewport(panel);
        assets.bindMaterials(materials);
        materials.setRevealHandler([&shell] { shell.dock()->showPanel("Material"); });

        McpServer mcpServer;
        McpPanel mcpPanel(mcpServer, settings);
        CHECK(mcpPanel.attach(shell.panel(EditorShell::kAiPanel)));
        McpContext context = { tree, document, selection, actions, panel, game, play, materials,
            assets, shell };
        McpTools tools(context);
        tools.registerTools(mcpServer);

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, settings, ui,
            panel, actions, menus, status, assets, materials, game, play, mcpServer, mcpPanel };
        CHECK(shell.setLayout("H200[{Hierarchy}|H780[V450[{Scene,Game/0}|{Console,Assets/1}]|"
                              "{Inspector,Material/0}]]"));
        rig.frames(4);

        CHECK(document.createProject(testRoot() + "/projects", "Mcp"));
        rig.frames(3);

        if (wanted("defaults")) testDefaults(rig);
        if (wanted("protocol")) testProtocol(rig);
        if (wanted("permissions")) testPermissions(rig);
        if (wanted("layout")) testLayout(rig);

        const char* const toolGroups[] = { "read", "scene", "materials", "import", "environment",
            "scenes", "code", "screenshots", "endtoend", "undo" };
        bool anyTools = false;
        for (const char* group: toolGroups) anyTools = anyTools || wanted(group);
        if (anyTools)
        {
            int port = 0;
            CHECK(startServer(rig, port));
            mcpPanel.editsSwitch()->setOn(true);
            mcpPanel.codeSwitch()->setOn(true);
            Client client(rig, port);
            CHECK(client.initialize());
            if (wanted("read")) testReadTools(rig, client);
            if (wanted("scene")) testSceneEdits(rig, client);
            if (wanted("materials")) testPrimitivesAndMaterials(rig, client);
            if (wanted("import")) testImport(rig, client);
            if (wanted("environment")) testEnvironment(rig, client);
            if (wanted("scenes")) testScenes(rig, client);
            if (wanted("code")) testCode(rig, client);
            if (wanted("screenshots")) testScreenshots(rig, client);
            if (wanted("endtoend")) testEndToEnd(rig, client);
            if (wanted("undo")) testUndoFromUi(rig, client);
            mcpPanel.stop();
            mcpPanel.editsSwitch()->setOn(false);
            mcpPanel.codeSwitch()->setOn(false);
        }
        if (wanted("panel")) testPanel(rig);
        if (wanted("shutdown")) testShutdown(rig);

        rig.frames(3);
        CHECK(!mcpServer.running());
        printf("mcp: %u requests logged, %s\n", mcpServer.logRevision(),
                useVulkan ? "vulkan" : "opengl");
        host.shutdown();
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);
    removeTree(testRoot());

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
