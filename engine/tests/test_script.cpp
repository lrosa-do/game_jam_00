#include "Check.h"

#include "prisma3d/Input.h"
#include "prisma3d/InputMap.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/ScriptInstance.h"
#include "prisma3d/script/ZenScripting.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b)
{
    return fabsf(a - b) < 0.001f;
}

void collectLine(const char* line, void* user)
{
    static_cast<ct::Vector<ct::String>*>(user)->push_back(ct::String(line));
}

ct::String joined(const ct::Vector<ct::String>& lines)
{
    ct::String text;
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        if (i > 0) text += ",";
        text += lines[i];
    }
    return text;
}

ct::String writeScript(const char* name, const ct::String& text)
{
    const ct::String path = ct::String(PRISMA3D_TEST_DIR) + "/script_tests/" + name;
    CHECK(FileSystem::writeText(path, text));
    return path;
}

struct Env
{
    Input input;
    Engine engine;
    SceneTree tree;
    ZenScripting scripting;
    ct::Vector<ct::String> log;

    Env()
    {
        registerSceneTypes();
        tree.initialize();
        scripting.setPrintHandler(&collectLine, &log);
        scripting.setHotReload(false);
    }

    Node* root()
    {
        return tree.getRoot();
    }

    void frame(double delta = 0.1)
    {
        input.beginFrame();
        tree.process(delta);
    }

    void physics(double delta = 0.1)
    {
        tree.physicsProcess(delta);
    }

    ct::String lines()
    {
        return joined(log);
    }

    int number(Node* node, const char* name)
    {
        bool valid = false;
        const PropertyValue value = node->get(name, &valid);
        CHECK(valid);
        return static_cast<int>(value.asInt());
    }

    float real(Node* node, const char* name)
    {
        bool valid = false;
        const PropertyValue value = node->get(name, &valid);
        CHECK(valid);
        return static_cast<float>(value.asFloat());
    }
};

const char kTracer[] = R"(class Tracer(Node3D):
    def _enter_tree(self):
        print("enter:" + self.name)
    def _ready(self):
        print("ready:" + self.name)
    def _process(self, delta):
        print("process:" + self.name)
    def _physics_process(self, delta):
        print("physics:" + self.name)
    def _exit_tree(self):
        print("exit:" + self.name)
)";

const char kAliases[] = R"(class Aliases(Node3D):
    def enter_tree(self):
        print("enter")
    def ready(self):
        print("ready")
    def process(self, delta):
        print("process")
    def physics_process(self, delta):
        print("physics")
    def input(self, event):
        print("input")
    def exit_tree(self):
        print("exit")
)";

const char kSpinner[] = R"(class Spinner(Node3D):
    speed = 2.0
    lives = 3
    label = "hero"
    enabled = True
    _hidden = 9
    def _ready(self):
        print("ready")
    def _process(self, delta):
        self.position.x += self.speed * delta
)";

InputEvent keyEvent(Key key, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = 0;
    return event;
}

void send(Env& env, const InputEvent& event)
{
    env.input.parseInputEvent(event);
    env.tree.inputEvent(event);
}

void testAttach()
{
    Env env;
    const ct::String path = writeScript("spinner.py", kSpinner);

    Node* node = env.root()->createChild("Node3D");
    CHECK(node->getScriptInstance() == nullptr);
    CHECK(node->getScript().empty());

    node->setScript(path);
    CHECK(node->getScript() == path);
    ScriptInstance* script = node->getScriptInstance();
    CHECK(script != nullptr);
    CHECK(script && script->hasScript());
    CHECK(script && script->getPath() == path);
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(env.scripting.isRunnable(node));
    CHECK(node->get("script").asString() == path);

    node->setScript("");
    CHECK(node->getScript().empty());
    CHECK(node->getScriptInstance() != nullptr);
    CHECK(!node->getScriptInstance()->hasScript());

    node->setScript(path);
    CHECK(node->getScriptInstance()->hasScript());
    CHECK(env.scripting.getCompileCount() == 1);

    Node* second = env.root()->createChild("Node3D");
    second->setScript(path);
    CHECK(env.scripting.getCompileCount() == 1);

    const ct::String cameraPath =
            writeScript("camera_only.py", "class CameraOnly(Camera3D):\n    zoom = 1.5\n");
    Node* plain = env.root()->createChild("Node3D");
    plain->setScript(cameraPath);
    CHECK(env.scripting.getErrorCount() == 1);
    CHECK(plain->getScriptInstance() == nullptr || !plain->getScriptInstance()->hasScript());
    CHECK(plain->getScript() == cameraPath);

    Node* camera = env.root()->createChild("Camera3D");
    camera->setScript(cameraPath);
    CHECK(camera->getScriptInstance() && camera->getScriptInstance()->hasScript());
    CHECK(env.scripting.getErrorCount() == 1);

    Node* mesh = env.root()->createChild("MeshInstance3D");
    mesh->setScript(path);
    CHECK(mesh->getScriptInstance() && mesh->getScriptInstance()->hasScript());

    const ct::String noClass = writeScript("no_class.py", "x = 1\n");
    Node* lonely = env.root()->createChild("Node3D");
    lonely->setScript(noClass);
    CHECK(env.scripting.getErrorCount() == 2);

    const ct::String missing = ct::String(PRISMA3D_TEST_DIR) + "/script_tests/does_not_exist.py";
    Node* absent = env.root()->createChild("Node3D");
    absent->setScript(missing);
    CHECK(env.scripting.getErrorCount() == 3);
    CHECK(absent->getScript() == missing);
}

void testLifecycleOrder()
{
    Env env;
    const ct::String path = writeScript("tracer.py", kTracer);

    Node holder;
    Node* a = holder.createChildWithScript("Node3D", path);
    a->setName("A");
    Node* b = a->createChildWithScript("Node3D", path);
    b->setName("B");
    Node* c = a->createChildWithScript("Node3D", path);
    c->setName("C");
    CHECK(env.log.empty());

    CHECK(a->reparent(env.root()));
    CHECK(env.lines() == "enter:A,enter:B,enter:C,ready:B,ready:C,ready:A");
    CHECK(a->isProcessing() && a->isPhysicsProcessing());

    env.log.clear();
    env.frame();
    CHECK(env.lines() == "process:A,process:B,process:C");

    env.log.clear();
    env.physics();
    CHECK(env.lines() == "physics:A,physics:B,physics:C");

    env.log.clear();
    CHECK(a->reparent(&holder));
    CHECK(env.lines() == "exit:C,exit:B,exit:A");

    env.log.clear();
    CHECK(a->reparent(env.root()));
    CHECK(env.lines() == "enter:A,enter:B,enter:C");
    CHECK(env.scripting.getErrorCount() == 0);
}

void testAliasHooks()
{
    Env env;
    const ct::String path = writeScript("aliases.py", kAliases);
    Node* node = env.root()->createChildWithScript("Node3D", path);
    CHECK(env.lines() == "enter,ready");
    env.log.clear();
    env.frame();
    env.physics();
    send(env, keyEvent(Key::A, true));
    CHECK(env.lines() == "process,physics,input");
    env.log.clear();
    node->queueFree();
    env.frame();
    CHECK(env.lines() == "process,exit");
    CHECK(env.scripting.getErrorCount() == 0);

    const ct::String wrong = writeScript("wrong_arity.py",
            "class WrongArity(Node3D):\n    def _process(self):\n        print(\"never\")\n"
            "    def _ready(self):\n        print(\"ready\")\n");
    env.log.clear();
    Node* other = env.root()->createChildWithScript("Node3D", wrong);
    CHECK(env.scripting.getErrorCount() >= 1);
    env.frame();
    CHECK(env.lines() == "ready");
    CHECK(other->getScriptInstance() != nullptr);
}

void testProperties()
{
    Env env;
    const ct::String path = writeScript("props.py",
            "class Props(Node3D):\n"
            "    def _ready(self):\n"
            "        self.position = Vec3(1, 2, 3)\n"
            "        self.position.x += 1\n"
            "        self.scale = Vec3(2, 2, 2)\n"
            "        self.visible = False\n"
            "        self.rotation_degrees = Vec3(0, 90, 0)\n"
            "        q = self.quaternion\n"
            "        self.tag = q.w\n"
            "        self.set(\"position\", self.get(\"position\") + Vec3(0, 10, 0))\n"
            "        self.size = self.global_position.y\n");
    Node3D* node = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(node->getPosition() == Math::Vec3(2.0f, 12.0f, 3.0f));
    CHECK(node->getScale() == Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(!node->isVisible());
    CHECK(near(node->getRotationDegrees().y, 90.0f));
    CHECK(near(node->getRotation().y, 1.5707964f));

    const ct::String lightPath = writeScript("light_props.py",
            "class LightProps(OmniLight3D):\n"
            "    def _ready(self):\n"
            "        self.light_energy = 4.5\n"
            "        c = self.light_color\n"
            "        c.r = 0.25\n"
            "        c.g = 0.5\n"
            "        self.light_color = c\n"
            "        self.set(\"light_energy\", self.get(\"light_energy\") * 2)\n");
    OmniLight3D* light =
            static_cast<OmniLight3D*>(env.root()->createChildWithScript("OmniLight3D", lightPath));
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(near(light->getEnergy(), 9.0f));
    CHECK(near(light->getColor().x, 0.25f));
    CHECK(near(light->getColor().y, 0.5f));

    const ct::String wrongType = writeScript("wrong_type.py", "class WrongType(Node3D):\n"
                                                              "    def _ready(self):\n"
                                                              "        self.position = 5\n");
    Node* bad = env.root()->createChildWithScript("Node3D", wrongType);
    CHECK(env.scripting.getErrorCount() == 1);
    CHECK(env.scripting.isFaulted(bad));
    CHECK(env.scripting.getLastError().contains("wrong_type.py:3"));

    const ct::String readBack = writeScript("read_back.py", "class ReadBack(Node3D):\n"
                                                            "    def _ready(self):\n"
                                                            "        self.name = \"renamed\"\n"
                                                            "        self.process_mode = 3\n");
    Node* renamed = env.root()->createChildWithScript("Node3D", readBack);
    CHECK(renamed->getName() == "renamed");
    CHECK(renamed->getProcessMode() == Node::ProcessMode::Always);
}

void testExports()
{
    Env env;
    const ct::String path = writeScript("spinner.py", kSpinner);
    Node3D* node = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    node->setName("Hero");

    ct::Vector<PropertyInfo> list;
    NodeClassDB::getInstancePropertyList(node, "Node", list, true);
    int group = -1;
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].isGroup() && list[i].name == "Script Variables") group = static_cast<int>(i);
    CHECK(group >= 0);
    bool speed = false;
    bool lives = false;
    bool label = false;
    bool enabled = false;
    bool hidden = false;
    for (std::size_t i = static_cast<std::size_t>(group < 0 ? 0 : group); i < list.size(); ++i)
    {
        if (list[i].name == "speed") speed = list[i].type == PropertyType::Float;
        if (list[i].name == "lives") lives = list[i].type == PropertyType::Int;
        if (list[i].name == "label") label = list[i].type == PropertyType::String;
        if (list[i].name == "enabled") enabled = list[i].type == PropertyType::Bool;
        if (list[i].name == "_hidden") hidden = true;
    }
    CHECK(speed && lives && label && enabled && !hidden);

    CHECK(env.real(node, "speed") == 2.0f);
    CHECK(env.number(node, "lives") == 3);
    CHECK(node->get("label").asString() == "hero");
    CHECK(node->get("enabled").asBool());

    CHECK(node->set("speed", PropertyValue::fromFloat(5.0)));
    CHECK(node->set("lives", PropertyValue::fromInt(7)));
    CHECK(node->set("label", PropertyValue::fromString("villain")));
    CHECK(node->set("enabled", PropertyValue::fromBool(false)));
    CHECK(node->set("speed", PropertyValue::fromInt(4)));
    CHECK(env.real(node, "speed") == 4.0f);
    CHECK(node->set("speed", PropertyValue::fromFloat(5.0)));
    CHECK(!node->set("speed", PropertyValue::fromString("x")));
    CHECK(!node->set("nothing_here", PropertyValue::fromInt(1)));

    env.frame(0.5);
    CHECK(near(node->getPosition().x, 2.5f));

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(node, text, &saved));
    CHECK(text.contains("\"script\""));
    CHECK(text.contains("\"speed\""));
    CHECK(text.contains("\"lives\""));
    CHECK(text.contains("\"label\""));
    CHECK(text.contains("\"enabled\""));
    CHECK(!text.contains("_hidden"));

    Node holder;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &holder, loaded);
    CHECK(copy != nullptr);
    CHECK(loaded.success());
    if (copy)
    {
        CHECK(copy->getScript() == path);
        CHECK(copy->getScriptInstance() && copy->getScriptInstance()->hasScript());
        CHECK(env.real(copy, "speed") == 5.0f);
        CHECK(env.number(copy, "lives") == 7);
        CHECK(copy->get("label").asString() == "villain");
        CHECK(!copy->get("enabled").asBool());
        ct::String again;
        CHECK(SceneSerializer::save(copy, again));
        CHECK(again == text);
    }

    Node3D* fresh = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    ct::String plain;
    CHECK(SceneSerializer::save(fresh, plain));
    CHECK(plain.contains("\"script\""));
    CHECK(!plain.contains("\"speed\""));
    CHECK(!plain.contains("\"lives\""));
    CHECK(env.scripting.getErrorCount() == 0);
}

void testTreeApi()
{
    Env env;
    const ct::String taggedPath = writeScript("tagged.py", "class Tagged(Node3D):\n"
                                                           "    def _ready(self):\n"
                                                           "        print(\"tagged ready\")\n");
    const ct::String spawnerPath = writeScript("spawner.py",
            ct::String("class Spawner(Node3D):\n"
                       "    age = 0\n"
                       "    def _ready(self):\n"
                       "        c = self.create_child(\"Node3D\")\n"
                       "        c.name = \"kid\"\n"
                       "        c.position = Vec3(0, 5, 0)\n"
                       "        t = self.create_child(\"MeshInstance3D\", \"") +
                    taggedPath +
                    "\")\n"
                    "        t.name = \"tagged\"\n"
                    "        self.kid = c\n"
                    "        print(\"same:\" + (\"yes\" if self.get_node(\"kid\") == c else "
                    "\"no\"))\n"
                    "        print(\"found:\" + self.find_child(\"tagged\").name)\n"
                    "        print(\"parent:\" + c.get_parent().name)\n"
                    "        print(\"count:\" + str(self.get_child_count()) + \"/\" + "
                    "str(len(self.get_children())))\n"
                    "        print(\"path:\" + c.get_path())\n"
                    "        print(\"class:\" + t.get_class() + \":\" + (\"yes\" if "
                    "t.is_class(\"Node3D\") else \"no\"))\n"
                    "    def _process(self, delta):\n"
                    "        self.age += 1\n"
                    "        if self.age == 2:\n"
                    "            self.kid.queue_free()\n"
                    "        print(\"valid:\" + (\"yes\" if is_valid(self.kid) else \"no\"))\n");

    Node holder;
    Node* spawner = holder.createChildWithScript("Node3D", spawnerPath);
    spawner->setName("Spawner");
    CHECK(spawner->reparent(env.root()));
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(env.lines() ==
            "tagged ready,same:yes,found:tagged,parent:Spawner,count:2/2,path:/root/Spawner/kid,"
            "class:MeshInstance3D:yes");
    CHECK(spawner->getChildCount() == 2);
    Node3D* kid = static_cast<Node3D*>(spawner->findChild("kid"));
    CHECK(kid && near(kid->getPosition().y, 5.0f));

    env.log.clear();
    env.frame();
    CHECK(env.lines() == "valid:yes");
    CHECK(spawner->findChild("kid") == kid);
    env.log.clear();
    env.frame();
    CHECK(env.lines() == "valid:yes");
    CHECK(spawner->findChild("kid") == nullptr);
    CHECK(spawner->getChildCount() == 1);
    env.log.clear();
    env.frame();
    CHECK(env.lines() == "valid:no");
    CHECK(env.scripting.getErrorCount() == 0);

    const ct::String stale =
            writeScript("stale.py", "class Stale(Node3D):\n"
                                    "    def _ready(self):\n"
                                    "        self.other = self.create_child(\"Node3D\")\n"
                                    "        self.other.queue_free()\n"
                                    "    def _process(self, delta):\n"
                                    "        print(self.other.name)\n");
    Node* stalest = env.root()->createChildWithScript("Node3D", stale);
    env.frame();
    env.frame();
    CHECK(env.scripting.getErrorCount() == 1);
    CHECK(env.scripting.isFaulted(stalest));

    const ct::String badClass =
            writeScript("bad_class.py", "class BadClass(Node3D):\n"
                                        "    def _ready(self):\n"
                                        "        self.create_child(\"NoSuchNode\")\n");
    Node* badder = env.root()->createChildWithScript("Node3D", badClass);
    CHECK(env.scripting.getErrorCount() == 2);
    CHECK(env.scripting.isFaulted(badder));
    CHECK(env.scripting.getLastError().contains("NoSuchNode"));
}

void testGroups()
{
    Env env;
    const ct::String member = writeScript("member.py", "class Member(Node3D):\n"
                                                       "    def _ready(self):\n"
                                                       "        self.add_to_group(\"team\")\n");
    const ct::String counter = writeScript("counter.py",
            "class Counter(Node3D):\n"
            "    def _process(self, delta):\n"
            "        nodes = self.get_tree().get_nodes_in_group(\"team\")\n"
            "        self.scale = Vec3(len(nodes), 1, 1)\n"
            "        self.first = self.get_tree().get_first_node_in_group(\"team\").name\n"
            "        self.flag = self.is_in_group(\"team\")\n"
            "        self.has = self.get_tree().has_group(\"team\")\n"
            "        self.total = self.get_tree().get_node_count_in_group(\"team\")\n");
    Node* one = env.root()->createChildWithScript("Node3D", member);
    one->setName("one");
    Node* two = env.root()->createChildWithScript("Node3D", member);
    two->setName("two");
    CHECK(one->isInGroup("team") && two->isInGroup("team"));
    CHECK(env.tree.getNodeCountInGroup("team") == 2);

    Node3D* watcher = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", counter));
    env.frame();
    CHECK(near(watcher->getScale().x, 2.0f));
    CHECK(!watcher->isInGroup("team"));

    one->queueFree();
    env.frame();
    env.frame();
    CHECK(near(watcher->getScale().x, 1.0f));
    CHECK(env.scripting.getErrorCount() == 0);
}

const char kPlayer[] = R"(class Player(Node3D):
    jumps = 0
    pressed = 0
    released = 0
    seen = 0
    handled = False
    mouse_x = 0.0
    def _process(self, delta):
        if action_check_pressed("jump"):
            self.jumps += 1
        if action_check("jump"):
            self.pressed += 1
        if action_check_released("jump"):
            self.released += 1
        v = action_vector("left", "right", "up", "down")
        self.position = Vec3(v.x, 0, v.y) + Vec3(action_axis("left", "right"), 0, 0)
        if keyboard_check(KEY_SPACE):
            self.scale = Vec3(2, 2, 2)
        if mouse_check_button(MOUSE_LEFT):
            self.rotation = Vec3(0, 1, 0)
        m = mouse_position()
        self.mouse_x = m.x
    def _input(self, event):
        if event.type == EVENT_KEY and event.is_action_pressed("jump"):
            self.seen += 1
        if event.type == EVENT_MOUSE_BUTTON:
            self.handled = True
            self.get_tree().set_input_as_handled()
)";

void testInput()
{
    Env env;
    InputMap& map = env.input.map();
    map.addAction("jump");
    map.actionAddEvent("jump", keyEvent(Key::Space, true));
    map.addAction("left");
    map.actionAddEvent("left", keyEvent(Key::A, true));
    map.addAction("right");
    map.actionAddEvent("right", keyEvent(Key::D, true));
    map.addAction("up");
    map.actionAddEvent("up", keyEvent(Key::W, true));
    map.addAction("down");
    map.actionAddEvent("down", keyEvent(Key::S, true));

    const ct::String second = writeScript("second_listener.py", "class SecondListener(Node3D):\n"
                                                                "    got = 0\n"
                                                                "    def _input(self, event):\n"
                                                                "        self.got += 1\n");
    Node* listener = env.root()->createChildWithScript("Node3D", second);

    const ct::String path = writeScript("player.py", kPlayer);
    Node3D* player = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    CHECK(player->isProcessingInput());
    CHECK(env.scripting.getErrorCount() == 0);

    env.input.beginFrame();
    send(env, keyEvent(Key::Space, true));
    send(env, keyEvent(Key::D, true));
    env.tree.process(0.1);
    CHECK(env.number(player, "jumps") == 1);
    CHECK(env.number(player, "pressed") == 1);
    CHECK(env.number(player, "seen") == 1);
    CHECK(near(player->getPosition().x, 2.0f));
    CHECK(near(player->getScale().x, 2.0f));

    env.frame();
    CHECK(env.number(player, "jumps") == 1);
    CHECK(env.number(player, "pressed") == 2);

    env.input.beginFrame();
    send(env, keyEvent(Key::Space, false));
    send(env, keyEvent(Key::D, false));
    send(env, keyEvent(Key::W, true));
    env.tree.process(0.1);
    CHECK(env.number(player, "released") == 1);
    CHECK(env.number(player, "pressed") == 2);
    CHECK(near(player->getPosition().z, -1.0f));
    CHECK(near(player->getPosition().x, 0.0f));

    env.input.beginFrame();
    InputEvent click;
    click.type = InputEventType::MouseButton;
    click.mouseButton.x = 40;
    click.mouseButton.y = 60;
    click.mouseButton.button = MouseButton::Left;
    click.mouseButton.pressed = true;
    click.mouseButton.modifiers = 0;
    send(env, click);
    env.tree.process(0.1);
    CHECK(player->get("handled").asBool());
    CHECK(near(player->getRotation().y, 1.0f));
    CHECK(env.real(player, "mouse_x") == 40.0f);

    const int heard = env.number(listener, "got");
    send(env, click);
    CHECK(env.number(listener, "got") == heard);
    send(env, keyEvent(Key::Q, true));
    CHECK(env.number(listener, "got") == heard + 1);
    CHECK(env.scripting.getErrorCount() == 0);
}

const char kCounter[] = R"(class Ticker(Node3D):
    ticks = 0
    phys = 0
    inputs = 0
    def _process(self, delta):
        self.ticks += 1
    def _physics_process(self, delta):
        self.phys += 1
    def _input(self, event):
        self.inputs += 1
)";

void testPauseAndProcessMode()
{
    Env env;
    const ct::String path = writeScript("ticker.py", kCounter);
    Node* normal = env.root()->createChildWithScript("Node3D", path);
    Node* always = env.root()->createChildWithScript("Node3D", path);
    always->setProcessMode(Node::ProcessMode::Always);
    Node* whenPaused = env.root()->createChildWithScript("Node3D", path);
    whenPaused->setProcessMode(Node::ProcessMode::WhenPaused);
    Node* disabled = env.root()->createChildWithScript("Node3D", path);
    disabled->setProcessMode(Node::ProcessMode::Disabled);

    for (int i = 0; i < 2; ++i)
    {
        env.frame();
        env.physics();
        send(env, keyEvent(Key::A, true));
    }
    CHECK(env.number(normal, "ticks") == 2 && env.number(normal, "phys") == 2 &&
            env.number(normal, "inputs") == 2);
    CHECK(env.number(always, "ticks") == 2 && env.number(always, "phys") == 2);
    CHECK(env.number(whenPaused, "ticks") == 0 && env.number(whenPaused, "phys") == 0 &&
            env.number(whenPaused, "inputs") == 0);
    CHECK(env.number(disabled, "ticks") == 0 && env.number(disabled, "inputs") == 0);

    env.tree.setPaused(true);
    for (int i = 0; i < 3; ++i)
    {
        env.frame();
        env.physics();
        send(env, keyEvent(Key::A, false));
    }
    CHECK(env.number(normal, "ticks") == 2 && env.number(normal, "phys") == 2 &&
            env.number(normal, "inputs") == 2);
    CHECK(env.number(always, "ticks") == 5 && env.number(always, "phys") == 5 &&
            env.number(always, "inputs") == 5);
    CHECK(env.number(whenPaused, "ticks") == 3 && env.number(whenPaused, "phys") == 3 &&
            env.number(whenPaused, "inputs") == 3);

    env.tree.setPaused(false);
    env.frame();
    CHECK(env.number(normal, "ticks") == 3);
    CHECK(env.number(whenPaused, "ticks") == 3);

    const ct::String pauser = writeScript("pauser.py",
            "class Pauser(Node3D):\n"
            "    def _ready(self):\n"
            "        self.process_mode = 3\n"
            "    def _process(self, delta):\n"
            "        self.get_tree().set_paused(not self.get_tree().is_paused())\n");
    env.root()->createChildWithScript("Node3D", pauser);
    env.frame();
    CHECK(env.tree.isPaused());
    env.frame();
    CHECK(!env.tree.isPaused());
    CHECK(env.scripting.getErrorCount() == 0);
}

// Closing the window: _close_requested runs on every scripted node; with auto accept quit off
// the tree keeps running, and a later close is heard again.
void testCloseRequest()
{
    Env env;
    const ct::String path = writeScript("closer.py",
            "class Closer(Node3D):\n"
            "    closes = 0\n"
            "    def _ready(self):\n"
            "        self.get_tree().set_auto_accept_quit(False)\n"
            "    def _close_requested(self):\n"
            "        self.closes += 1\n"
            "        if self.closes == 2:\n"
            "            self.get_tree().quit()\n");
    Node* closer = env.root()->createChildWithScript("Node3D", path);
    env.frame();
    CHECK(!env.tree.isAutoAcceptQuit());
    env.tree.notification(Node::kNotificationWmCloseRequest);
    CHECK(env.number(closer, "closes") == 1);
    CHECK(!env.tree.process(0.1));
    env.tree.notification(Node::kNotificationWmCloseRequest);
    CHECK(env.number(closer, "closes") == 2);
    CHECK(env.tree.process(0.1));
    CHECK(env.scripting.getErrorCount() == 0);
}

const char kToolScript[] = R"(class ToolScript(Node3D):
    tool = True
    ticks = 0
    def __init__(self):
        print("tool init")
    def _ready(self):
        print("tool ready")
    def _process(self, delta):
        self.ticks += 1
)";

const char kPlainScript[] = R"(class PlainScript(Node3D):
    ticks = 0
    speed = 1.5
    def __init__(self):
        print("plain init")
    def _ready(self):
        print("plain ready")
    def _process(self, delta):
        self.ticks += 1
)";

void testEditorHint()
{
    Env env;
    CHECK(!env.engine.isEditorHint());
    env.engine.setEditorHint(true);
    CHECK(env.engine.isEditorHint());

    const ct::String toolPath = writeScript("tool_script.py", kToolScript);
    const ct::String plainPath = writeScript("plain_script.py", kPlainScript);
    Node* tool = env.root()->createChildWithScript("Node3D", toolPath);
    Node* plain = env.root()->createChildWithScript("Node3D", plainPath);
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(env.lines() == "tool init,tool ready");
    CHECK(env.scripting.isRunnable(tool));
    CHECK(!env.scripting.isRunnable(plain));

    env.log.clear();
    env.frame();
    env.frame();
    CHECK(env.number(tool, "ticks") == 2);
    CHECK(env.number(plain, "ticks") == 0);
    CHECK(env.lines().empty());

    ct::Vector<PropertyInfo> list;
    NodeClassDB::getInstancePropertyList(plain, "Node", list, true);
    bool listed = false;
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].name == "speed") listed = true;
    CHECK(listed);
    CHECK(plain->set("speed", PropertyValue::fromFloat(3.0)));
    CHECK(env.real(plain, "speed") == 3.0f);

    env.engine.setEditorHint(false);
    CHECK(env.scripting.isRunnable(plain));
    env.frame();
    CHECK(env.lines() == "plain init");
    CHECK(env.number(plain, "ticks") == 1);
    CHECK(env.number(tool, "ticks") == 3);
    CHECK(env.real(plain, "speed") == 3.0f);
}

void testErrorIsolation()
{
    Env env;
    const ct::String bad = writeScript("bad.py", "class Bad(Node3D):\n"
                                                 "    def _process(self, delta):\n"
                                                 "        x = None\n"
                                                 "        x.boom()\n");
    const ct::String good = writeScript("good.py", "class Good(Node3D):\n"
                                                   "    ticks = 0\n"
                                                   "    def _process(self, delta):\n"
                                                   "        self.ticks += 1\n");
    Node* first = env.root()->createChildWithScript("Node3D", good);
    Node* culprit = env.root()->createChildWithScript("Node3D", bad);
    Node* last = env.root()->createChildWithScript("Node3D", good);
    culprit->setName("culprit");

    for (int i = 0; i < 4; ++i) env.frame();
    CHECK(env.number(first, "ticks") == 4);
    CHECK(env.number(last, "ticks") == 4);
    CHECK(env.scripting.getErrorCount() == 1);
    CHECK(env.scripting.isFaulted(culprit));
    CHECK(!env.scripting.isFaulted(first));
    CHECK(!env.scripting.isRunnable(culprit));
    CHECK(env.scripting.getLastError().contains("bad.py:4"));
    CHECK(culprit->isInsideTree());

    const ct::String initFails = writeScript("init_fails.py", "class InitFails(Node3D):\n"
                                                              "    def __init__(self):\n"
                                                              "        y = None\n"
                                                              "        y.boom()\n"
                                                              "    def _ready(self):\n"
                                                              "        print(\"unreachable\")\n");
    Node* broken = env.root()->createChildWithScript("Node3D", initFails);
    CHECK(env.scripting.isFaulted(broken));
    CHECK(env.scripting.getErrorCount() == 2);
    CHECK(env.lines().empty());
    env.frame();
    CHECK(env.number(first, "ticks") == 5);

    const ct::String syntax = writeScript("syntax.py", "class Syntax(Node3D:\n    pass\n");
    Node* typo = env.root()->createChildWithScript("Node3D", syntax);
    CHECK(env.scripting.getErrorCount() == 3);
    CHECK(env.scripting.getLastError().contains("syntax.py:"));
    CHECK(typo->getScriptInstance() == nullptr || !typo->getScriptInstance()->hasScript());

    const ct::String topLevel = writeScript("top_level.py",
            "class TopLevel(Node3D):\n    pass\nundefined_function()\n");
    Node* eager = env.root()->createChildWithScript("Node3D", topLevel);
    CHECK(env.scripting.getErrorCount() == 4);
    CHECK(eager->getScriptInstance() == nullptr || !eager->getScriptInstance()->hasScript());
    env.frame();
    CHECK(env.number(last, "ticks") == 6);
}

void testHotReload()
{
    Env env;
    const char* version1 = "class Mover(Node3D):\n"
                           "    speed = 1.0\n"
                           "    ticks = 0\n"
                           "    def _process(self, delta):\n"
                           "        self.ticks += 1\n"
                           "        self.position.x += self.speed\n";
    const char* version2 = "class Mover(Node3D):\n"
                           "    speed = 5.0\n"
                           "    extra = 7\n"
                           "    ticks = 0\n"
                           "    def _process(self, delta):\n"
                           "        self.ticks += 1\n"
                           "        self.position.x += self.speed * 10\n";
    const ct::String path = writeScript("mover.py", version1);
    Node3D* untouched = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    Node3D* tweaked = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    CHECK(tweaked->set("speed", PropertyValue::fromFloat(3.0)));
    CHECK(env.scripting.getCompileCount() == 1);

    env.frame();
    env.frame();
    CHECK(near(untouched->getPosition().x, 2.0f));
    CHECK(near(tweaked->getPosition().x, 6.0f));
    CHECK(env.scripting.reloadChanged() == 0);

    writeScript("mover.py", version2);
    CHECK(env.scripting.reloadChanged() == 1);
    CHECK(env.scripting.getCompileCount() == 2);
    CHECK(env.scripting.reloadChanged() == 0);
    CHECK(env.number(untouched, "ticks") == 2);
    CHECK(env.number(tweaked, "ticks") == 2);
    CHECK(env.real(untouched, "speed") == 5.0f);
    CHECK(env.real(tweaked, "speed") == 3.0f);
    CHECK(env.number(untouched, "extra") == 7);

    env.frame();
    CHECK(near(untouched->getPosition().x, 52.0f));
    CHECK(near(tweaked->getPosition().x, 36.0f));
    CHECK(env.number(untouched, "ticks") == 3);

    writeScript("mover.py", "class Mover(Node3D:\n    pass  # broken on purpose\n");
    CHECK(env.scripting.reloadChanged() == 0);
    CHECK(env.scripting.getErrorCount() == 1);
    CHECK(env.scripting.reloadChanged() == 0);
    CHECK(env.scripting.getErrorCount() == 1);
    env.frame();
    CHECK(near(untouched->getPosition().x, 102.0f));

    const ct::String fixed = "class Mover(Node3D):\n"
                             "    speed = 2.0\n"
                             "    ticks = 0\n"
                             "    def _process(self, delta):\n"
                             "        self.ticks += 1\n"
                             "        self.position.x += 100\n"
                             "        # fixed\n";
    writeScript("mover.py", fixed);
    CHECK(env.scripting.reloadChanged() == 1);
    env.frame();
    CHECK(near(untouched->getPosition().x, 202.0f));

    const char* faulty = "class Mover(Node3D):\n"
                         "    ticks = 0\n"
                         "    def _process(self, delta):\n"
                         "        self.nothing.explode()\n";
    writeScript("mover.py", faulty);
    CHECK(env.scripting.reloadChanged() == 1);
    env.frame();
    CHECK(env.scripting.isFaulted(untouched));
    CHECK(env.scripting.getErrorCount() == 3);

    writeScript("mover.py", version1);
    CHECK(env.scripting.reloadChanged() == 1);
    CHECK(!env.scripting.isFaulted(untouched));
    env.frame();
    CHECK(!env.scripting.isFaulted(untouched));

    env.scripting.setHotReload(true, 0.0);
    CHECK(env.scripting.isHotReload());
    writeScript("mover.py",
            "class Mover(Node3D):\n    speed = 9.0\n    def _ready(self):\n        pass\n");
    const int compiles = env.scripting.getCompileCount();
    env.frame();
    CHECK(env.scripting.getCompileCount() == compiles + 1);
}

void testScene()
{
    Env env;
    const ct::String mover = writeScript("scene_mover.py",
            "class SceneMover(Node3D):\n"
            "    speed = 1.0\n"
            "    next_scene = \"\"\n"
            "    frames = 0\n"
            "    def _process(self, delta):\n"
            "        self.position.x += self.speed * delta\n"
            "        self.frames += 1\n"
            "        if self.frames == 12 and self.next_scene != \"\":\n"
            "            self.get_tree().change_scene(self.next_scene)\n");
    const ct::String arrived = writeScript("scene_arrived.py", "class Arrived(Node3D):\n"
                                                               "    def _ready(self):\n"
                                                               "        print(\"arrived\")\n"
                                                               "        self.get_tree().quit(3)\n");

    const ct::String second = ct::String(PRISMA3D_TEST_DIR) + "/script_tests/second.scene.json";
    const ct::String secondText =
            ct::String(
                    "{\"format\":\"prisma3d-scene\",\"version\":1,\"root\":{\"class\":\"Node3D\","
                    "\"name\":\"Level2\",\"children\":[{\"class\":\"Node3D\",\"name\":\"Goal\","
                    "\"properties\":{\"script\":\"") +
            arrived + "\"}}]}}";
    CHECK(FileSystem::writeText(second, secondText));

    const ct::String firstText =
            ct::String(
                    "{\"format\":\"prisma3d-scene\",\"version\":1,\"root\":{\"class\":\"Node3D\","
                    "\"name\":\"Level1\",\"children\":[{\"class\":\"Node3D\",\"name\":\"Mover\","
                    "\"properties\":{\"script\":\"") +
            mover + "\",\"speed\":3.0,\"next_scene\":\"" + second + "\"}}]}}";

    SceneResult result;
    Node* level = SceneSerializer::load(firstText, env.root(), result);
    CHECK(level != nullptr);
    CHECK(result.success());
    if (!level) return;
    env.tree.setCurrentScene(level);
    Node3D* mobile = static_cast<Node3D*>(level->findChild("Mover"));
    CHECK(mobile != nullptr);
    if (!mobile) return;
    CHECK(env.real(mobile, "speed") == 3.0f);

    for (int i = 0; i < 10; ++i) env.frame();
    CHECK(near(mobile->getPosition().x, 3.0f));
    CHECK(env.scripting.getErrorCount() == 0);

    ct::String saved;
    CHECK(SceneSerializer::save(level, saved));
    CHECK(saved.contains("\"speed\""));
    CHECK(saved.contains("scene_mover.py"));
    CHECK(saved.contains("next_scene"));

    env.frame();
    CHECK(env.tree.getCurrentScene() == level);
    env.frame();
    CHECK(env.tree.isSceneChangePending());
    CHECK(env.lines().empty());
    const bool quit = env.tree.process(0.1);
    CHECK(!env.tree.isSceneChangePending());
    CHECK(env.tree.getCurrentScene() != nullptr);
    CHECK(env.tree.getCurrentScene() && env.tree.getCurrentScene()->getName() == "Level2");
    CHECK(env.root()->findChild("Level1") == nullptr);
    CHECK(env.lines() == "arrived");
    CHECK(quit || env.tree.process(0.1));
    CHECK(env.tree.getExitCode() == 3);
    CHECK(env.scripting.getErrorCount() == 0);
}

void testGlobals()
{
    Env env;
    const ct::String path =
            writeScript("globals.py", "import math\n"
                                      "class Globals(Node3D):\n"
                                      "    def _ready(self):\n"
                                      "        t = current_time()\n"
                                      "        self.ok = t >= 0\n"
                                      "        self.hint = is_editor()\n"
                                      "        self.scale_value = time_scale()\n"
                                      "        q = Quat(0, 0, 0, 1)\n"
                                      "        self.quaternion = q\n"
                                      "        v = Vec3(1, 2, 3) * 2\n"
                                      "        self.position = v + Vec3(math.sqrt(16), 0, 0)\n"
                                      "        c = Color(1, 0.5, 0.25)\n"
                                      "        self.alpha = c.a\n"
                                      "        self.key = KEY_W\n"
                                      "        self.has_action = action_exists(\"jump\")\n"
                                      "        action_add(\"jump\")\n"
                                      "        action_add_key(\"jump\", KEY_SPACE)\n"
                                      "        self.has_after = action_exists(\"jump\")\n");
    Node3D* node = static_cast<Node3D*>(env.root()->createChildWithScript("Node3D", path));
    CHECK(env.scripting.getErrorCount() == 0);
    CHECK(node->getPosition() == Math::Vec3(6.0f, 4.0f, 6.0f));
    CHECK(env.input.map().hasAction("jump"));
    const auto* events = env.input.map().actionGetEvents("jump");
    CHECK(events != nullptr && events->size() == 1);
    CHECK(node->getQuaternion().w == 1.0f);

    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    ct::HashMap<ct::String, int> names;
    const char* reserved[] = { "ready", "process", "input", "enter_tree", "exit_tree",
        "physics_process", "_ready", "_process", "_input", "_enter_tree", "_exit_tree",
        "_physics_process" };
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
        ct::Vector<PropertyInfo> list;
        NodeClassDB::getPropertyList(classes[i], list, true);
        for (std::size_t p = 0; p < list.size(); ++p)
        {
            if (list[p].isGroup()) continue;
            names.put(list[p].name, 1);
            for (const char* name: reserved) CHECK(!(list[p].name == name));
        }
    }
    CHECK(names.size() < 1024);
}

void testGarbageCollection()
{
    Env env;
    const char* first = "class Stress(Node3D):\n"
                        "    n = 0\n"
                        "    def _ready(self):\n"
                        "        self.kid = self.create_child(\"Node3D\")\n"
                        "        self.light = self.create_child(\"OmniLight3D\")\n"
                        "    def _process(self, delta):\n"
                        "        collect()\n"
                        "        self.n += 1\n"
                        "        s = \"frame\" + str(self.n)\n"
                        "        self.kid.position = Vec3(self.n, 0, 0)\n"
                        "        self.kid.name = s\n"
                        "        c = self.light.light_color\n"
                        "        c.b = 0.5\n"
                        "        self.light.light_color = c\n"
                        "        l = [1, 2, 3, s]\n"
                        "        self.label = s + str(len(l))\n"
                        "        self.first = self.get_tree().get_root().get_child(0).name\n"
                        "    def _input(self, event):\n"
                        "        collect()\n"
                        "        self.last = event.type\n";
    const ct::String path = writeScript("stress.py", first);
    Node* stress = env.root()->createChildWithScript("Node3D", path);
    for (int i = 0; i < 20; ++i)
    {
        env.frame();
        send(env, keyEvent(Key::A, i % 2 == 0));
    }
    CHECK(env.number(stress, "n") == 20);
    Node3D* kid = static_cast<Node3D*>(stress->getChild(0));
    CHECK(near(kid->getPosition().x, 20.0f));
    CHECK(kid->getName() == "frame20");
    CHECK(env.scripting.getErrorCount() == 0);

    const char* second = "class Stress(Node3D):\n"
                         "    n = 0\n"
                         "    m = 5\n"
                         "    def _process(self, delta):\n"
                         "        collect()\n"
                         "        self.n += self.m\n"
                         "        self.get_child(0).position = Vec3(self.n, 1, 1)\n";
    writeScript("stress.py", second);
    CHECK(env.scripting.reloadChanged() == 1);
    for (int i = 0; i < 5; ++i) env.frame();
    CHECK(env.number(stress, "n") == 45);
    CHECK(near(kid->getPosition().x, 45.0f));
    CHECK(env.scripting.getErrorCount() == 0);
}

} // namespace

int main()
{
    testAttach();
    testLifecycleOrder();
    testAliasHooks();
    testProperties();
    testExports();
    testTreeApi();
    testGroups();
    testInput();
    testPauseAndProcessMode();
    testCloseRequest();
    testEditorHint();
    testErrorIsolation();
    testHotReload();
    testScene();
    testGlobals();
    testGarbageCollection();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
