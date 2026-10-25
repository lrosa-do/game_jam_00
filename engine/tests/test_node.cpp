#include "Check.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kCustom = 5000;

int code(int id, int what)
{
    return id * 10000 + what;
}

class RecordingNode : public Node
{
public:
    static int live;

    RecordingNode(int nodeId, ct::Vector<int>* nodeSink = nullptr)
            : id(nodeId),
              sink(nodeSink),
              processCount(0),
              physicsCount(0),
              lastDelta(0.0)
    {
        ++live;
    }

    ~RecordingNode() override
    {
        --live;
    }

    int count(int what) const
    {
        int total = 0;
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            if (events[i] == what) ++total;
        }
        return total;
    }

    int id;
    ct::Vector<int>* sink;
    ct::Vector<int> events;
    int processCount;
    int physicsCount;
    double lastDelta;

protected:
    void notificationHandler(int what) override
    {
        events.push_back(what);
        if (what == kNotificationProcess)
        {
            ++processCount;
            lastDelta = getProcessDeltaTime();
        }
        else if (what == kNotificationPhysicsProcess)
        {
            ++physicsCount;
            lastDelta = getPhysicsProcessDeltaTime();
        }
        bool lifecycle = what == kNotificationEnterTree || what == kNotificationPostEnterTree ||
                         what == kNotificationReady || what == kNotificationExitTree ||
                         what == kCustom;
        if (sink && lifecycle) sink->push_back(code(id, what));
        Node::notificationHandler(what);
    }
};

int RecordingNode::live = 0;

bool sequence(const ct::Vector<int>& a, const ct::Vector<int>& b)
{
    if (a.size() != b.size()) return false;
    std::size_t i = 0;
    for (int value: b)
    {
        if (a[i++] != value) return false;
    }
    return true;
}

void testTreeBuilding()
{
    Node a;
    Node holder;
    a.setName("a");

    CHECK(a.getName() == "a");
    CHECK(a.getChildCount() == 0);
    CHECK(a.getParent() == nullptr);
    CHECK(a.getIndex() == -1);
    CHECK(!a.isInsideTree());
    CHECK(a.getTree() == nullptr);

    Node* b = a.createChild<Node>();
    Node* c = a.createChild<Node>();
    Node* d = a.createChild<Node>();
    b->setName("b");
    c->setName("c");
    d->setName("d");

    CHECK(a.getChildCount() == 3);
    CHECK(a.getChild(0) == b);
    CHECK(a.getChild(1) == c);
    CHECK(a.getChild(2) == d);
    CHECK(a.getChild(3) == nullptr);
    CHECK(b->getParent() == &a);
    CHECK(b->getIndex() == 0);
    CHECK(c->getIndex() == 1);
    CHECK(d->getIndex() == 2);

    CHECK(c->reparent(&holder));
    CHECK(a.getChildCount() == 2);
    CHECK(c->getParent() == &holder);
    CHECK(c->getIndex() == 0);
    CHECK(d->getIndex() == 1);
    CHECK(a.getChild(1) == d);

    CHECK(!c->reparent(&holder));
    CHECK(a.getChildCount() == 2);

    CHECK(b->reparent(&holder));
    CHECK(d->reparent(&holder));
    CHECK(a.getChildCount() == 0);
    CHECK(holder.getChildCount() == 3);
}

bool isGenerated(const Node* node, const char* className)
{
    ct::String prefix = "@";
    prefix += className;
    prefix += '@';
    const ct::String& name = node->getName();
    if (name.size() <= prefix.size() || strncmp(name.c_str(), prefix.c_str(), prefix.size()) != 0)
        return false;
    for (std::size_t i = prefix.size(); i < name.size(); ++i)
        if (name[i] < '0' || name[i] > '9') return false;
    return true;
}

long generatedNumber(const Node* node)
{
    const ct::String& name = node->getName();
    const std::size_t at = name.find('@', 1);
    return at == ct::String::npos ? -1 : atol(name.c_str() + at + 1);
}

Node* named(Node& parent, const char* wanted)
{
    Node* child = parent.createChild<Node>();
    child->setName(wanted);
    return child;
}

void testGeneratedNames()
{
    Node parent;
    Node* first = parent.createChild<Node>();
    Node* second = parent.createChild<Node>();
    Node3D* third = parent.createChild<Node3D>();
    CHECK(isGenerated(first, "Node"));
    CHECK(isGenerated(second, "Node"));
    CHECK(isGenerated(third, "Node3D"));
    CHECK(first->getName() != second->getName());
    CHECK(generatedNumber(first) >= 2);
    CHECK(generatedNumber(second) > generatedNumber(first));
    CHECK(generatedNumber(third) > generatedNumber(second));
    CHECK(parent.findChild(first->getName()) == first);
    CHECK(parent.findChild(third->getName()) == third);

    Node loose;
    CHECK(loose.getName().empty());

    Node* readable = parent.createChild("Node", true);
    Node* readableSecond = parent.createChild("Node", true);
    Node* readableThird = parent.createChild("Node", true);
    CHECK(readable && readable->getName() == "Node");
    CHECK(readableSecond && readableSecond->getName() == "Node2");
    CHECK(readableThird && readableThird->getName() == "Node3");
    Node* spatial = parent.createChild("Node3D", true);
    CHECK(spatial && spatial->getName() == "Node3D");
    CHECK(parent.createChild("Node3D", true)->getName() == "Node3D2");

    Node holder;
    Node* clash = named(holder, "clash");
    Node* newcomer = holder.createChild<Node>();
    newcomer->setName("clash");
    CHECK(newcomer->getName() == "clash2");
    CHECK(clash->getName() == "clash");

    Node removable;
    Node elsewhere;
    Node* gone = named(removable, "gone");
    CHECK(gone->reparent(&elsewhere));
    CHECK(removable.findChild("gone") == nullptr);
    CHECK(gone->getName() == "gone");
    CHECK(elsewhere.findChild("gone") == gone);
    Node* stand = named(removable, "gone");
    CHECK(stand->getName() == "gone");
    CHECK(gone->reparent(&removable));
    CHECK(isGenerated(gone, "Node"));
    CHECK(removable.findChild(gone->getName()) == gone);
    CHECK(removable.findChild("gone") == stand);
    CHECK(elsewhere.findChild("gone") == nullptr);
}

void testSerialNames()
{
    Node parent;
    Node* a = named(parent, "Node");
    Node* b = named(parent, "Node");
    Node* c = named(parent, "Node");
    CHECK(a->getName() == "Node");
    CHECK(b->getName() == "Node2");
    CHECK(c->getName() == "Node3");

    CHECK(named(parent, "Node2")->getName() == "Node4");
    CHECK(named(parent, "Node3")->getName() == "Node5");
    CHECK(named(parent, "Node5")->getName() == "Node6");

    struct Case
    {
        const char* first;
        const char* second;
        const char* expected;
    };
    const Case cases[] = {
        { "Item09", "Item09", "Item10" },
        { "Item007", "Item007", "Item008" },
        { "Item009", "Item009", "Item010" },
        { "Item99", "Item99", "Item100" },
        { "Item0999", "Item0999", "Item1000" },
        { "Item000", "Item000", "Item001" },
        { "Item0", "Item0", "Item1" },
        { "Item5", "Item5", "Item6" },
        { "123", "123", "124" },
        { "9", "9", "10" },
        { "Name_7", "Name_7", "Name_8" },
        { "Mixed1a", "Mixed1a", "Mixed1a2" },
        { "Mixed1a2", "Mixed1a2", "Mixed1a3" },
    };
    for (const Case& test: cases)
    {
        Node group;
        named(group, test.first);
        Node* other = named(group, test.second);
        if (other->getName() != test.expected)
        {
            printf("serial name '%s' -> '%s', expected '%s'\n", test.second,
                    other->getName().c_str(), test.expected);
            ++failures;
        }
        CHECK(group.getChildCount() == 2);
        CHECK(group.findChild(test.expected) == other);
    }

    Node skip;
    named(skip, "Item09");
    named(skip, "Item10");
    named(skip, "Item11");
    CHECK(named(skip, "Item09")->getName() == "Item12");

    Node many;
    for (int i = 1; i <= 300; ++i)
    {
        Node* child = named(many, "n");
        ct::String expected = "n";
        if (i > 1) expected.append_number(i);
        CHECK(child->getName() == expected);
    }
    CHECK(many.getChildCount() == 300);
    CHECK(many.findChild("n300") == many.getChild(299));
}

void testSeparators()
{
    CHECK(Node::getNameNumSeparator() == Node::NameNumSeparator::None);

    Node::setNameNumSeparator(Node::NameNumSeparator::Space);
    {
        Node parent;
        named(parent, "Node");
        CHECK(named(parent, "Node")->getName() == "Node 2");
        CHECK(named(parent, "Node")->getName() == "Node 3");
        CHECK(named(parent, "Node 2")->getName() == "Node 4");
        named(parent, "Node5");
        CHECK(named(parent, "Node5")->getName() == "Node5 2");
        named(parent, "Pad 007");
        CHECK(named(parent, "Pad 007")->getName() == "Pad 008");
        CHECK(parent.createChild("Node", true)->getName() == "Node 5");
    }
    Node::setNameNumSeparator(Node::NameNumSeparator::Underscore);
    {
        Node parent;
        named(parent, "Node");
        CHECK(named(parent, "Node")->getName() == "Node_2");
        CHECK(named(parent, "Node_2")->getName() == "Node_3");
        named(parent, "Node7");
        CHECK(named(parent, "Node7")->getName() == "Node7_2");
    }
    Node::setNameNumSeparator(Node::NameNumSeparator::Dash);
    {
        Node parent;
        named(parent, "Node");
        CHECK(named(parent, "Node")->getName() == "Node-2");
        CHECK(named(parent, "Node-2")->getName() == "Node-3");
        named(parent, "5");
        CHECK(named(parent, "5")->getName() == "5-2");
    }
    Node::setNameNumSeparator(Node::NameNumSeparator::None);
    CHECK(Node::getNameNumSeparator() == Node::NameNumSeparator::None);
}

void testSetNameAmongSiblings()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    Node* a = named(*root, "a");
    Node* b = named(*root, "b");
    Node* c = named(*root, "c");
    CHECK(root->findChild("a") == a && root->findChild("b") == b && root->findChild("c") == c);

    b->setName("a");
    CHECK(b->getName() == "a2");
    CHECK(a->getName() == "a");
    CHECK(root->findChild("a") == a);
    CHECK(root->findChild("a2") == b);
    CHECK(root->findChild("b") == nullptr);
    CHECK(root->getNode("a2") == b);
    CHECK(b->getPath() == "/root/a2");

    c->setName("b");
    CHECK(c->getName() == "b");
    CHECK(root->findChild("b") == c);
    CHECK(root->findChild("c") == nullptr);

    a->setName("a");
    CHECK(a->getName() == "a");
    b->setName("a2");
    CHECK(b->getName() == "a2");
    CHECK(root->findChild("a2") == b);

    a->setName("a2");
    CHECK(a->getName() == "a3");
    CHECK(root->findChild("a") == nullptr);
    CHECK(root->findChild("a3") == a);

    c->setName("");
    CHECK(c->getName() == "b");
    CHECK(root->findChild("b") == c);
    Node loose;
    loose.setName("");
    CHECK(loose.getName().empty());

    c->setName("a.b/c:d@e\"f%g");
    CHECK(c->getName() == "a_b_c_d_e_f_g");
    CHECK(root->findChild("a_b_c_d_e_f_g") == c);
    CHECK(root->getNode("a_b_c_d_e_f_g") == c);
    CHECK(root->findChild("b") == nullptr);
    c->setName("..");
    CHECK(c->getName() == "__");
    CHECK(Node::validateNodeName("plain name 1") == "plain name 1");
    CHECK(Node::validateNodeName(".:@/\"%") == "______");
    CHECK(Node::validateNodeName("").empty());

    Node* internal = root->createChild<Node>();
    internal->setInternal(true);
    internal->setName("a3");
    CHECK(internal->getName() == "a4");
    CHECK(root->findChild("a3") == a);

    Node parent;
    Node* detached = named(parent, "same");
    Node* sibling = named(parent, "other");
    sibling->setName("same");
    CHECK(sibling->getName() == "same2");
    detached->setName("same2");
    CHECK(detached->getName() == "same3");
    CHECK(parent.findChild("same") == nullptr);
}

void testReparentNames()
{
    Node first;
    Node second;
    Node* mover = named(first, "x");
    Node* resident = named(second, "x");
    Node* other = named(second, "y");

    CHECK(mover->reparent(&second));
    CHECK(isGenerated(mover, "Node"));
    CHECK(resident->getName() == "x");
    CHECK(other->getName() == "y");
    CHECK(second.getChildCount() == 3);
    CHECK(second.findChild("x") == resident);
    CHECK(second.findChild(mover->getName()) == mover);
    CHECK(first.getChildCount() == 0);
    CHECK(first.findChild("x") == nullptr);

    Node* again = named(first, "x");
    CHECK(again->getName() == "x");
    Node* free = named(first, "free");
    CHECK(free->reparent(&second));
    CHECK(free->getName() == "free");
    CHECK(second.findChild("free") == free);
    CHECK(first.findChild("free") == nullptr);

    Node* unrelated = named(first, "y");
    CHECK(unrelated->reparent(&second));
    CHECK(unrelated->getName() != "y");
    CHECK(!unrelated->getName().empty());
    CHECK(second.findChild("y") == other);

    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    Node* inTree = named(*root, "shared");
    Node* branch = named(*root, "branch");
    Node* branchShared = named(*branch, "shared");
    CHECK(branchShared->reparent(root));
    CHECK(branchShared->getName() != "shared");
    CHECK(branchShared->getParent() == root);
    CHECK(root->findChild("shared") == inTree);
    CHECK(root->getNode(branchShared->getName()) == branchShared);
    CHECK(branch->findChild("shared") == nullptr);
}

void testReparentReadableNames()
{
    Node first;
    Node second;
    Node* resident = named(second, "x");
    Node* mover = named(first, "x");

    CHECK(mover->reparent(&second, true));
    CHECK(mover->getName() == "x2");
    CHECK(resident->getName() == "x");
    CHECK(second.findChild("x2") == mover);
    CHECK(first.getChildCount() == 0);

    Node* third = named(first, "x");
    CHECK(third->reparent(&second, true));
    CHECK(third->getName() == "x3");
    CHECK(second.getChildCount() == 3);

    Node* free = named(first, "free");
    CHECK(free->reparent(&second, true));
    CHECK(free->getName() == "free");

    Node* sameParent = second.findChild("free");
    CHECK(!sameParent->reparent(&second, true));
    CHECK(!sameParent->reparent(sameParent, true));
    CHECK(!second.reparent(sameParent, true));
    CHECK(!sameParent->reparent(nullptr, true));
    CHECK(sameParent->getName() == "free");
}

void testAddChildRejected()
{
    Node a;
    Node other;
    Node* b = a.createChild<Node>();
    Node* c = b->createChild<Node>();

    CHECK(!a.reparent(nullptr));
    CHECK(a.getChildCount() == 1);

    CHECK(!a.reparent(&a));
    CHECK(a.getChildCount() == 1);
    CHECK(a.getParent() == nullptr);

    CHECK(!a.reparent(c));
    CHECK(c->getChildCount() == 0);
    CHECK(a.getParent() == nullptr);

    CHECK(!a.reparent(b));
    CHECK(b->getChildCount() == 1);
    CHECK(a.getParent() == nullptr);

    CHECK(!b->reparent(&a));
    CHECK(a.getChildCount() == 1);

    CHECK(b->reparent(&other));
    CHECK(other.getChildCount() == 1);
    CHECK(a.getChildCount() == 0);
    CHECK(b->getParent() == &other);

    CHECK(b->reparent(&a));
    CHECK(a.getChildCount() == 1);
    CHECK(other.getChildCount() == 0);
}

void testMoveChild()
{
    Node a;
    Node* nodes[4];
    RecordingNode* recording[4];
    for (int i = 0; i < 4; ++i)
    {
        recording[i] = a.createChild<RecordingNode>(i + 1);
        nodes[i] = recording[i];
    }

    a.moveChild(nodes[3], 0);
    CHECK(a.getChild(0) == nodes[3]);
    CHECK(a.getChild(1) == nodes[0]);
    CHECK(a.getChild(2) == nodes[1]);
    CHECK(a.getChild(3) == nodes[2]);
    for (int i = 0; i < 4; ++i) CHECK(a.getChild(static_cast<std::size_t>(i))->getIndex() == i);

    a.moveChild(nodes[3], 4);
    CHECK(a.getChild(3) == nodes[3]);
    CHECK(a.getChild(0) == nodes[0]);
    for (int i = 0; i < 4; ++i) CHECK(a.getChild(static_cast<std::size_t>(i))->getIndex() == i);

    a.moveChild(nodes[3], 9);
    CHECK(a.getChild(3) == nodes[3]);

    recording[0]->events.clear();
    recording[1]->events.clear();
    a.moveChild(nodes[0], 2);
    CHECK(a.getChild(2) == nodes[0]);
    CHECK(a.getChild(0) == nodes[1]);
    CHECK(recording[0]->count(Node::kNotificationMovedInParent) == 1);
    CHECK(recording[1]->count(Node::kNotificationMovedInParent) == 0);

    a.moveChild(nodes[0], 2);
    CHECK(recording[0]->count(Node::kNotificationMovedInParent) == 1);

    Node outsider;
    a.moveChild(&outsider, 0);
    CHECK(a.getChildCount() == 4);
}

void testFindAndPaths()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    Node* a = root->createChild<Node>();
    Node* x = root->createChild<Node>();
    Node* b = a->createChild<Node>();
    Node* c = b->createChild<Node>();
    a->setName("a");
    x->setName("x");
    b->setName("b");
    c->setName("c");

    CHECK(root->findChild("a") == a);
    CHECK(root->findChild("x") == x);
    CHECK(root->findChild("b") == nullptr);
    CHECK(a->findChild("b") == b);

    CHECK(root->getNode("a") == a);
    CHECK(root->getNode("a/b/c") == c);
    CHECK(a->getNode("b/c") == a->getChild(0)->getChild(0));
    CHECK(c->getNode("..") == b);
    CHECK(c->getNode("../..") == a);
    CHECK(b->getNode("../../x") == x);
    CHECK(c->getNode("../../../x") == x);
    CHECK(c->getNode(".") == c);
    CHECK(c->getNode("./") == c);
    CHECK(a->getNode("./b/./c") == c);
    CHECK(a->getNode("b/../b") == b);
    CHECK(a->getNode("") == nullptr);
    CHECK(a->getNode("missing") == nullptr);
    CHECK(a->getNode("b/missing") == nullptr);
    CHECK(root->getNode("..") == nullptr);
    CHECK(c->getNode("../../../..") == nullptr);

    CHECK(c->getNode("/root") == root);
    CHECK(c->getNode("/root/a/b/c") == c);
    CHECK(a->getNode("/root/x") == x);
    CHECK(a->getNode("/wrong/a") == nullptr);
    CHECK(a->getNode("/root/nothing") == nullptr);
    CHECK(a->getNode("/") == nullptr);

    CHECK(root->getPath() == "/root");
    CHECK(a->getPath() == "/root/a");
    CHECK(c->getPath() == "/root/a/b/c");

    Node detached;
    Node* detachedChild = detached.createChild<Node>();
    detached.setName("d");
    detachedChild->setName("e");
    CHECK(detachedChild->getPath().empty());
    CHECK(detachedChild->getNode("/d/e") == nullptr);
    CHECK(detached.getNode("e") == detachedChild);
    CHECK(detachedChild->getNode("..") == &detached);

    CHECK(root->isAncestorOf(c));
    CHECK(a->isAncestorOf(c));
    CHECK(b->isAncestorOf(c));
    CHECK(!c->isAncestorOf(a));
    CHECK(!a->isAncestorOf(a));
    CHECK(!x->isAncestorOf(c));
    CHECK(!a->isAncestorOf(nullptr));
    CHECK(!a->isAncestorOf(&detached));
}

void testClasses()
{
    Node plain;
    Node3D spatial;
    Node* asNode = &spatial;

    CHECK(plain.isClass(NodeClass::Node));
    CHECK(!plain.isClass(NodeClass::Node3D));
    CHECK(spatial.isClass(NodeClass::Node));
    CHECK(spatial.isClass(NodeClass::Node3D));
    CHECK(!spatial.isClass("Camera3D"));

    CHECK(asNode->castTo<Node3D>() == &spatial);
    CHECK(asNode->castTo<Node>() == asNode);
    CHECK(plain.castTo<Node3D>() == nullptr);
    CHECK(plain.castTo<Node>() == &plain);

    const Node* constNode = &spatial;
    CHECK(constNode->castTo<Node3D>() == &spatial);
    const Node* constPlain = &plain;
    CHECK(constPlain->castTo<Node3D>() == nullptr);
}

void testLifecycleOrder()
{
    ct::Vector<int> sink;
    RecordingNode::live = 0;

    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    CHECK(root != nullptr);
    CHECK(root->getName() == "root");
    CHECK(root->isInsideTree());
    CHECK(root->isReady());
    CHECK(root->getTree() == &tree);
    CHECK(tree.getNodeCount() == 1);
    CHECK(SceneTree::singleton() == &tree);

    Node holder;
    RecordingNode* a = holder.createChild<RecordingNode>(1, &sink);
    RecordingNode* b = a->createChild<RecordingNode>(2, &sink);
    RecordingNode* c = b->createChild<RecordingNode>(3, &sink);
    CHECK(sink.empty());
    CHECK(!a->isInsideTree());
    CHECK(!c->isReady());

    a->events.clear();
    b->events.clear();
    c->events.clear();

    CHECK(a->reparent(root));

    CHECK(sequence(sink,
            { code(1, Node::kNotificationEnterTree), code(2, Node::kNotificationEnterTree),
                code(3, Node::kNotificationEnterTree), code(3, Node::kNotificationPostEnterTree),
                code(3, Node::kNotificationReady), code(2, Node::kNotificationPostEnterTree),
                code(2, Node::kNotificationReady), code(1, Node::kNotificationPostEnterTree),
                code(1, Node::kNotificationReady) }));
    CHECK(sequence(a->events, { Node::kNotificationUnparented, Node::kNotificationParented,
                                  Node::kNotificationEnterTree, Node::kNotificationPostEnterTree,
                                  Node::kNotificationReady }));
    CHECK(c->isInsideTree());
    CHECK(c->isReady());
    CHECK(c->getTree() == &tree);
    CHECK(tree.getNodeCount() == 4);

    sink.clear();
    CHECK(a->reparent(&holder));
    CHECK(sequence(sink,
            { code(3, Node::kNotificationExitTree), code(2, Node::kNotificationExitTree),
                code(1, Node::kNotificationExitTree) }));
    CHECK(!a->isInsideTree());
    CHECK(!c->isInsideTree());
    CHECK(!c->isReady());
    CHECK(c->getTree() == nullptr);
    CHECK(tree.getNodeCount() == 1);
    CHECK(a->events[a->events.size() - 3] == Node::kNotificationExitTree);
    CHECK(a->events[a->events.size() - 2] == Node::kNotificationUnparented);
    CHECK(a->events[a->events.size() - 1] == Node::kNotificationParented);

    sink.clear();
    CHECK(a->reparent(root));
    CHECK(sequence(sink,
            { code(1, Node::kNotificationEnterTree), code(2, Node::kNotificationEnterTree),
                code(3, Node::kNotificationEnterTree), code(3, Node::kNotificationPostEnterTree),
                code(2, Node::kNotificationPostEnterTree),
                code(1, Node::kNotificationPostEnterTree) }));
    CHECK(a->count(Node::kNotificationReady) == 1);
    CHECK(c->count(Node::kNotificationReady) == 1);
    CHECK(c->count(Node::kNotificationEnterTree) == 2);
    CHECK(c->isReady());
    CHECK(tree.getNodeCount() == 4);

    sink.clear();
    RecordingNode* late = c->createChild<RecordingNode>(4, &sink);
    CHECK(sequence(sink,
            { code(4, Node::kNotificationEnterTree), code(4, Node::kNotificationPostEnterTree),
                code(4, Node::kNotificationReady) }));
    CHECK(late->isReady());
    CHECK(tree.getNodeCount() == 5);

    sink.clear();
    a->propagateNotification(kCustom);
    CHECK(sequence(sink,
            { code(1, kCustom), code(2, kCustom), code(3, kCustom), code(4, kCustom) }));
    sink.clear();
    b->propagateNotification(kCustom);
    CHECK(sequence(sink, { code(2, kCustom), code(3, kCustom), code(4, kCustom) }));
    sink.clear();
    late->notification(kCustom);
    CHECK(sequence(sink, { code(4, kCustom) }));

    CHECK(RecordingNode::live == 4);
}

void testProcessing()
{
    RecordingNode::live = 0;

    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    RecordingNode* both = root->createChild<RecordingNode>(1);
    RecordingNode* idleOnly = root->createChild<RecordingNode>(2);
    RecordingNode* physicsOnly = root->createChild<RecordingNode>(3);
    RecordingNode* none = root->createChild<RecordingNode>(4);
    RecordingNode* nested = none->createChild<RecordingNode>(5);

    CHECK(!both->isProcessing());
    CHECK(!both->isPhysicsProcessing());

    both->setProcess(true);
    both->setPhysicsProcess(true);
    idleOnly->setProcess(true);
    physicsOnly->setPhysicsProcess(true);
    nested->setProcess(true);
    CHECK(both->isProcessing());
    CHECK(both->isPhysicsProcessing());
    CHECK(idleOnly->isProcessing());
    CHECK(!idleOnly->isPhysicsProcessing());

    CHECK(tree.getFrame() == 0);
    CHECK(!tree.process(0.25));
    CHECK(tree.getFrame() == 1);
    CHECK(both->processCount == 1);
    CHECK(idleOnly->processCount == 1);
    CHECK(physicsOnly->processCount == 0);
    CHECK(none->processCount == 0);
    CHECK(nested->processCount == 1);
    CHECK(both->physicsCount == 0);
    CHECK(both->lastDelta == 0.25);
    CHECK(nested->lastDelta == 0.25);

    CHECK(!tree.physicsProcess(0.5));
    CHECK(both->physicsCount == 1);
    CHECK(physicsOnly->physicsCount == 1);
    CHECK(idleOnly->physicsCount == 0);
    CHECK(none->physicsCount == 0);
    CHECK(both->lastDelta == 0.5);
    CHECK(physicsOnly->lastDelta == 0.5);
    CHECK(both->getProcessDeltaTime() == 0.25);
    CHECK(both->getPhysicsProcessDeltaTime() == 0.5);
    CHECK(tree.getFrame() == 1);

    idleOnly->setProcess(false);
    CHECK(!tree.process(0.125));
    CHECK(idleOnly->processCount == 1);
    CHECK(both->processCount == 2);
    CHECK(both->lastDelta == 0.125);
    CHECK(tree.getFrame() == 2);

    CHECK(!tree.isPaused());
    tree.setPaused(true);
    CHECK(tree.isPaused());
    CHECK(both->count(Node::kNotificationPaused) == 1);
    CHECK(nested->count(Node::kNotificationPaused) == 1);
    tree.process(0.1);
    tree.physicsProcess(0.1);
    CHECK(both->processCount == 2);
    CHECK(both->physicsCount == 1);
    tree.setPaused(true);
    CHECK(both->count(Node::kNotificationPaused) == 1);
    tree.setPaused(false);
    CHECK(both->count(Node::kNotificationUnpaused) == 1);
    tree.process(0.1);
    CHECK(both->processCount == 3);

    Node detached;
    CHECK(detached.getProcessDeltaTime() == 0.0);
    CHECK(detached.getPhysicsProcessDeltaTime() == 0.0);
}

void testProcessModifiedDuringTraversal()
{
    class Remover : public RecordingNode
    {
    public:
        Remover(int nodeId, Node* nodeTarget)
                : RecordingNode(nodeId),
                  target(nodeTarget)
        {
        }

    protected:
        void notificationHandler(int what) override
        {
            RecordingNode::notificationHandler(what);
            if (what == kNotificationProcess && getParent() != target) reparent(target);
        }

    private:
        Node* target;
    };

    RecordingNode::live = 0;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    {
        Node holder;
        Remover* first = root->createChild<Remover>(1, &holder);
        RecordingNode* second = root->createChild<RecordingNode>(2);
        first->setProcess(true);
        second->setProcess(true);

        tree.process(0.1);
        CHECK(first->processCount == 1);
        CHECK(second->processCount == 1);
        CHECK(first->getParent() == &holder);
        CHECK(root->getChildCount() == 1);
    }
    CHECK(RecordingNode::live == 1);
}

// The tree walk skips subtrees with nothing to process: a node deep under idle parents still
// processes after it is turned on, moved to another branch, or its branch is detached and added
// back; turning it off stops it.
void testProcessSkipsIdleSubtrees()
{
    RecordingNode::live = 0;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    RecordingNode* a = root->createChild<RecordingNode>(1);
    RecordingNode* b = a->createChild<RecordingNode>(2);
    RecordingNode* deep = b->createChild<RecordingNode>(3);
    RecordingNode* other = root->createChild<RecordingNode>(4);

    tree.process(0.1);
    tree.physicsProcess(0.1);
    CHECK(deep->processCount == 0);

    deep->setProcess(true);
    deep->setPhysicsProcess(true);
    tree.process(0.1);
    tree.physicsProcess(0.1);
    CHECK(deep->processCount == 1);
    CHECK(deep->physicsCount == 1);

    // Moved with its parent to another branch.
    CHECK(b->reparent(other));
    tree.process(0.1);
    CHECK(deep->processCount == 2);

    // Detached, then added back under the first branch.
    other->removeChild(b);
    a->addChild(b);
    tree.process(0.1);
    tree.physicsProcess(0.1);
    CHECK(deep->processCount == 3);
    CHECK(deep->physicsCount == 2);

    // Turning it on twice and off once leaves it off.
    deep->setProcess(true);
    deep->setProcess(false);
    tree.process(0.1);
    CHECK(deep->processCount == 3);
    tree.physicsProcess(0.1);
    CHECK(deep->physicsCount == 3);

    // A node created under a processing branch, and the processing parent stopped.
    a->setProcess(true);
    RecordingNode* late = b->createChild<RecordingNode>(5);
    late->setProcess(true);
    a->setProcess(false);
    tree.process(0.1);
    CHECK(late->processCount == 1);
    CHECK(a->processCount == 0);
}

void testQueueFree()
{
    RecordingNode::live = 0;

    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    RecordingNode* keep = root->createChild<RecordingNode>(1);
    RecordingNode* doomed = root->createChild<RecordingNode>(2);
    RecordingNode* doomedChild = doomed->createChild<RecordingNode>(3);
    (void) doomedChild;
    CHECK(tree.getNodeCount() == 4);
    CHECK(RecordingNode::live == 3);
    CHECK(!doomed->isQueuedForDeletion());

    doomed->queueFree();
    doomed->queueFree();
    CHECK(doomed->isQueuedForDeletion());
    CHECK(RecordingNode::live == 3);
    CHECK(tree.getNodeCount() == 4);
    CHECK(root->getChildCount() == 2);

    tree.process(0.1);
    CHECK(RecordingNode::live == 1);
    CHECK(tree.getNodeCount() == 2);
    CHECK(root->getChildCount() == 1);
    CHECK(root->getChild(0) == keep);

    RecordingNode* parent = root->createChild<RecordingNode>(4);
    RecordingNode* child = parent->createChild<RecordingNode>(5);
    child->queueFree();
    parent->queueFree();
    CHECK(RecordingNode::live == 3);
    tree.physicsProcess(0.1);
    CHECK(RecordingNode::live == 1);
    CHECK(tree.getNodeCount() == 2);

    Node holder;
    RecordingNode* loose = holder.createChild<RecordingNode>(6);
    loose->queueFree();
    CHECK(loose->isQueuedForDeletion());
    CHECK(RecordingNode::live == 2);
    tree.process(0.1);
    CHECK(RecordingNode::live == 1);

    {
        Node manualHolder;
        RecordingNode* manual = manualHolder.createChild<RecordingNode>(7);
        manual->queueFree();
        CHECK(RecordingNode::live == 2);
    }
    CHECK(RecordingNode::live == 1);
    tree.process(0.1);
    CHECK(RecordingNode::live == 1);
    CHECK(tree.getNodeCount() == 2);
}

void testQuit()
{
    {
        SceneTree tree;
        tree.initialize();
        CHECK(tree.isAutoAcceptQuit());
        CHECK(!tree.process(0.1));
        CHECK(!tree.physicsProcess(0.1));
        CHECK(tree.getExitCode() == 0);
        tree.quit(3);
        CHECK(tree.getExitCode() == 3);
        CHECK(tree.process(0.1));
        CHECK(tree.physicsProcess(0.1));
    }

    {
        RecordingNode::live = 0;
        SceneTree tree;
        tree.initialize();
        RecordingNode* listener = tree.getRoot()->createChild<RecordingNode>(1);

        tree.setAutoAcceptQuit(false);
        CHECK(!tree.isAutoAcceptQuit());
        tree.notification(Node::kNotificationWmCloseRequest);
        CHECK(listener->count(Node::kNotificationWmCloseRequest) == 1);
        CHECK(!tree.process(0.1));

        tree.setAutoAcceptQuit(true);
        tree.notification(Node::kNotificationWmCloseRequest);
        CHECK(listener->count(Node::kNotificationWmCloseRequest) == 2);
        CHECK(tree.process(0.1));
    }

    {
        RecordingNode::live = 0;
        SceneTree tree;
        tree.initialize();
        RecordingNode* listener = tree.getRoot()->createChild<RecordingNode>(1);
        tree.notification(MainLoop::kNotificationApplicationFocusOut);
        CHECK(listener->count(MainLoop::kNotificationApplicationFocusOut) == 1);
        CHECK(!tree.process(0.1));
    }
}

void testDestruction()
{
    RecordingNode::live = 0;

    {
        Node holder;
        RecordingNode* a = holder.createChild<RecordingNode>(1);
        RecordingNode* b = a->createChild<RecordingNode>(2);
        b->createChild<RecordingNode>(3);
        a->createChild<RecordingNode>(4);
        CHECK(RecordingNode::live == 4);
    }
    CHECK(RecordingNode::live == 0);

    {
        SceneTree tree;
        tree.initialize();
        Node* parent = tree.getRoot()->createChild<Node>();
        RecordingNode* child = parent->createChild<RecordingNode>(1);
        RecordingNode* other = parent->createChild<RecordingNode>(2);
        child->queueFree();
        tree.process(0.1);
        CHECK(parent->getChildCount() == 1);
        CHECK(parent->getChild(0) == other);
        CHECK(other->getIndex() == 0);
        CHECK(RecordingNode::live == 1);
    }
    CHECK(RecordingNode::live == 0);

    {
        SceneTree tree;
        tree.initialize();
        RecordingNode* a = tree.getRoot()->createChild<RecordingNode>(1);
        a->createChild<RecordingNode>(2);
        CHECK(tree.getNodeCount() == 3);

        RecordingNode* c = tree.getRoot()->createChild<RecordingNode>(3);
        CHECK(tree.getNodeCount() == 4);
        c->queueFree();
        tree.process(0.1);
        CHECK(tree.getNodeCount() == 3);
        CHECK(tree.getRoot()->getChildCount() == 1);

        a->queueFree();
        tree.process(0.1);
        CHECK(tree.getNodeCount() == 1);
        CHECK(RecordingNode::live == 0);

        RecordingNode* d = tree.getRoot()->createChild<RecordingNode>(4);
        d->createChild<RecordingNode>(5);
        CHECK(RecordingNode::live == 2);
    }
    CHECK(RecordingNode::live == 0);
    CHECK(SceneTree::singleton() == nullptr);
}

void testFinalize()
{
    RecordingNode::live = 0;
    SceneTree tree;
    tree.initialize();
    RecordingNode* a = tree.getRoot()->createChild<RecordingNode>(1);
    a->queueFree();
    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    CHECK(tree.getNodeCount() == 0);
    CHECK(RecordingNode::live == 0);
}

} // namespace

int main()
{
    testTreeBuilding();
    testGeneratedNames();
    testSerialNames();
    testSeparators();
    testSetNameAmongSiblings();
    testReparentNames();
    testReparentReadableNames();
    testAddChildRejected();
    testMoveChild();
    testFindAndPaths();
    testClasses();
    testLifecycleOrder();
    testProcessing();
    testProcessModifiedDuringTraversal();
    testProcessSkipsIdleSubtrees();
    testQueueFree();
    testQuit();
    testDestruction();
    testFinalize();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
