#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;
const float kPi = 3.14159265358979f;

bool near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

void run(SceneTree& tree, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i) tree.physicsProcess(kStep);
}

StaticBody3D* makeFrame(Node* root, const char* name, const Math::Vec3& position)
{
    StaticBody3D* body = root->createChild<StaticBody3D>();
    body->setName(name);
    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(0.5f, 0.5f, 0.5f));
    body->setPosition(position);
    return body;
}

RigidBody3D* makeBox(Node* root, const char* name, const Math::Vec3& position,
        const Math::Vec3& size = Math::Vec3(0.5f, 0.5f, 0.5f))
{
    RigidBody3D* body = root->createChild<RigidBody3D>();
    body->setName(name);
    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    shape->setBox(size);
    body->setPosition(position);
    body->setCanSleep(false);
    return body;
}

float distanceBetween(Node3D* a, Node3D* b)
{
    return (a->getGlobalPosition() - b->getGlobalPosition()).Length();
}

void testHingeDoor()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    makeFrame(root, "Frame", Math::Vec3(0.0f, 0.0f, 0.0f));
    RigidBody3D* door =
            makeBox(root, "Door", Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 0.2f, 0.2f));
    HingeJoint3D* hinge = root->createChild<HingeJoint3D>();
    hinge->setNodeA("../Frame");
    hinge->setNodeB("../Door");
    CHECK(hinge->getJoint().valid() || !hinge->isConfigured());
    tree.physicsProcess(kStep);
    CHECK(hinge->isConfigured());
    CHECK(hinge->getExcludeNodesFromCollision());
    CHECK(hinge->getBodyA() != nullptr && hinge->getBodyB() == door);

    hinge->setUseLimit(true);
    hinge->setLimit(-0.5f, 0.5f);
    run(tree, 2.0);
    printf("  hinge: angle %.3f, door at %.2f %.2f\n", hinge->getAngle(),
            door->getGlobalPosition().x, door->getGlobalPosition().y);
    CHECK(near(hinge->getAngle(), -0.5f, 0.06f));
    CHECK(near((door->getGlobalPosition() - Math::Vec3(0.0f, 0.0f, 0.0f)).Length(), 1.0f, 0.05f));

    hinge->setUseLimit(false);
    run(tree, 1.0);
    CHECK(hinge->getAngle() < -0.9f);

    tree.finalize();
}

void testSliderAndMotor()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* rail = makeFrame(root, "Rail", Math::Vec3(0.0f, 5.0f, 0.0f));
    rail->setCollisionLayer(0);
    rail->setCollisionMask(0);
    RigidBody3D* car = makeBox(root, "Car", Math::Vec3(0.0f, 5.0f, 0.0f));
    SliderJoint3D* slider = root->createChild<SliderJoint3D>();
    slider->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    slider->setBodies(rail, car);
    tree.physicsProcess(kStep);
    CHECK(slider->isConfigured());
    slider->setUseLimit(true);
    slider->setLimit(-1.0f, 2.0f);
    car->setLinearVelocity(Math::Vec3(5.0f, 1.0f, 0.0f));
    run(tree, 2.0);
    printf("  slider: position %.3f, car %.2f %.2f\n", slider->getPosition(),
            car->getGlobalPosition().x, car->getGlobalPosition().y);
    CHECK(near(slider->getPosition(), 2.0f, 0.1f));
    CHECK(near(car->getGlobalPosition().y, 5.0f, 0.05f));

    slider->setUseLimit(false);
    slider->setMotor(-2.0f, 1000.0f);
    run(tree, 3.0);
    CHECK(slider->getPosition() < -1.0f);
    slider->disableMotor();

    tree.finalize();
}

void testPinPendulum()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* pivot = makeFrame(root, "Pivot", Math::Vec3(0.0f, 5.0f, 0.0f));
    RigidBody3D* bob = makeBox(root, "Bob", Math::Vec3(1.5f, 5.0f, 0.0f));
    PinJoint3D* pin = root->createChild<PinJoint3D>();
    pin->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    pin->setBodies(pivot, bob);
    float low = 1e9f;
    float high = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        tree.physicsProcess(kStep);
        const float d = distanceBetween(bob, pivot);
        if (d < low) low = d;
        if (d > high) high = d;
    }
    printf("  pin: distance %.3f to %.3f, bob y %.2f\n", low, high, bob->getGlobalPosition().y);
    CHECK(low > 1.4f && high < 1.6f);
    CHECK(bob->getGlobalPosition().y < 5.0f);

    tree.finalize();
}

void testFixedAndDistance()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* anchor = makeFrame(root, "Anchor", Math::Vec3(0.0f, 10.0f, 0.0f));
    RigidBody3D* first = makeBox(root, "First", Math::Vec3(1.0f, 10.0f, 0.0f));
    RigidBody3D* second = makeBox(root, "Second", Math::Vec3(2.0f, 10.0f, 0.0f));
    FixedJoint3D* weld = root->createChild<FixedJoint3D>();
    weld->setPosition(Math::Vec3(0.5f, 10.0f, 0.0f));
    weld->setBodies(anchor, first);
    FixedJoint3D* weld2 = root->createChild<FixedJoint3D>();
    weld2->setPosition(Math::Vec3(1.5f, 10.0f, 0.0f));
    weld2->setBodies(first, second);

    StaticBody3D* hook = makeFrame(root, "Hook", Math::Vec3(10.0f, 10.0f, 0.0f));
    RigidBody3D* bob = makeBox(root, "Bob", Math::Vec3(12.0f, 10.0f, 0.0f));
    DistanceJoint3D* rope = root->createChild<DistanceJoint3D>();
    rope->setPosition(Math::Vec3(10.0f, 10.0f, 0.0f));
    rope->setOffsetB(Math::Vec3(2.0f, 0.0f, 0.0f));
    rope->setBodies(hook, bob);
    tree.physicsProcess(kStep);
    CHECK(weld->isConfigured() && weld2->isConfigured() && rope->isConfigured());
    rope->setDistance(2.0f, 2.0f);

    float low = 1e9f;
    float high = 0.0f;
    for (int i = 0; i < 180; ++i)
    {
        tree.physicsProcess(kStep);
        const float d = distanceBetween(bob, hook);
        if (d < low) low = d;
        if (d > high) high = d;
    }
    printf("  fixed: first (%.3f %.3f), second (%.3f %.3f); rope %.3f to %.3f\n",
            first->getGlobalPosition().x, first->getGlobalPosition().y,
            second->getGlobalPosition().x, second->getGlobalPosition().y, low, high);
    CHECK(near(first->getGlobalPosition().y, 10.0f, 0.05f));
    CHECK(near(second->getGlobalPosition().y, 10.0f, 0.08f));
    CHECK(low > 1.9f && high < 2.1f);

    tree.finalize();
}

void testPistonUniversalWheelMouse()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* baseA = makeFrame(root, "BaseA", Math::Vec3(0.0f, 5.0f, 0.0f));
    baseA->setCollisionLayer(0);
    baseA->setCollisionMask(0);
    RigidBody3D* rod = makeBox(root, "Rod", Math::Vec3(0.0f, 5.0f, 0.0f));
    PistonJoint3D* piston = root->createChild<PistonJoint3D>();
    piston->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    piston->setBodies(baseA, rod);

    StaticBody3D* baseB = makeFrame(root, "BaseB", Math::Vec3(10.0f, 5.0f, 0.0f));
    baseB->setCollisionLayer(0);
    baseB->setCollisionMask(0);
    RigidBody3D* arm = makeBox(root, "Arm", Math::Vec3(10.0f, 5.0f, 1.0f));
    UniversalJoint3D* universal = root->createChild<UniversalJoint3D>();
    universal->setPosition(Math::Vec3(10.0f, 5.0f, 0.0f));
    universal->setBodies(baseB, arm);

    StaticBody3D* chassis = makeFrame(root, "Chassis", Math::Vec3(20.0f, 5.0f, 0.0f));
    chassis->setCollisionLayer(0);
    chassis->setCollisionMask(0);
    RigidBody3D* wheel = makeBox(root, "Wheel", Math::Vec3(20.0f, 4.0f, 0.0f));
    wheel->setMass(15.0f);
    WheelJoint3D* wheelJoint = root->createChild<WheelJoint3D>();
    wheelJoint->setPosition(Math::Vec3(20.0f, 4.0f, 0.0f));
    wheelJoint->setBodies(chassis, wheel);

    RigidBody3D* ball = makeBox(root, "Ball", Math::Vec3(30.0f, 5.0f, 0.0f));
    MouseJoint3D* mouse = root->createChild<MouseJoint3D>();
    mouse->setPosition(Math::Vec3(30.0f, 5.0f, 0.0f));
    mouse->setBodies(ball, nullptr);

    tree.physicsProcess(kStep);
    CHECK(piston->isConfigured());
    CHECK(universal->isConfigured());
    CHECK(wheelJoint->isConfigured());
    CHECK(mouse->isConfigured());

    piston->setLinearLimit(-1.0f, 2.0f);
    piston->setLinearMotor(3.0f, 1000.0f);
    universal->setMotorX(1.0f, 100.0f);
    wheelJoint->setSuspension(0.0f, 2000.0f, 200.0f);
    wheelJoint->setSteeringServo(0.3f, 500.0f, 5.0f);
    mouse->setSpring(2000.0f, 5.0f, 1.0f);
    mouse->setTarget(Math::Vec3(36.0f, 8.0f, 0.0f));
    run(tree, 1.0);
    wheelJoint->setSpinMotor(6.0f, 500.0f);
    run(tree, 2.0);

    printf("  piston %.3f, universal %.3f/%.3f, wheel steering %.3f travel %.3f, ball %.2f %.2f\n",
            piston->getPosition(), universal->getAngleX(), universal->getAngleY(),
            wheelJoint->getSteeringAngle(), wheelJoint->getSuspensionTravel(),
            ball->getGlobalPosition().x, ball->getGlobalPosition().y);
    CHECK(near(piston->getPosition(), 2.0f, 0.1f));
    CHECK(fabsf(universal->getAngleX()) > 0.4f);
    CHECK(fabsf(universal->getAngleY()) < 0.1f);
    CHECK(near(wheelJoint->getSteeringAngle(), 0.3f, 0.05f));
    CHECK(fabsf(wheelJoint->getSuspensionTravel()) < 0.05f);
    CHECK((ball->getGlobalPosition() - Math::Vec3(36.0f, 8.0f, 0.0f)).Length() < 0.6f);

    tree.finalize();
}

void buildCart(Node* root, WheelJoint3D* wheels[4], RigidBody3D** chassis)
{
    RigidBody3D* body = root->createChild<RigidBody3D>();
    CollisionShape3D* bodyShape = body->createChild<CollisionShape3D>();
    bodyShape->setBox(Math::Vec3(2.4f, 0.5f, 1.4f));
    body->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    body->setMass(40.0f);
    body->setCanSleep(false);
    for (int i = 0; i < 4; ++i)
    {
        const float x = (i & 1) ? 0.9f : -0.9f;
        const float z = (i & 2) ? 0.85f : -0.85f;
        RigidBody3D* wheel = root->createChild<RigidBody3D>();
        CollisionShape3D* shape = wheel->createChild<CollisionShape3D>();
        shape->setSphere(0.4f);
        wheel->setPosition(Math::Vec3(x, 0.4f, z));
        wheel->setMass(8.0f);
        wheel->setCanSleep(false);
        wheel->setFriction(2.0f);
        WheelJoint3D* joint = root->createChild<WheelJoint3D>();
        joint->setPosition(Math::Vec3(x, 0.4f, z));
        joint->setRotationDegrees(Math::Vec3(0.0f, 90.0f, 0.0f));
        joint->setBodies(body, wheel);
        joint->setSuspension(0.0f, 4000.0f, 300.0f);
        joint->setSteeringLimit(-0.5f, 0.5f);
        wheels[i] = joint;
    }
    *chassis = body;
}

float headingOf(const RigidBody3D* body)
{
    const Math::Mat4 frame = body->getGlobalTransform();
    return atan2f(-frame.col0.z, frame.col0.x) * 57.29578f;
}

void testCart()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* groundShape = ground->createChild<CollisionShape3D>();
    groundShape->setBox(Math::Vec3(200.0f, 1.0f, 200.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    WheelJoint3D* wheels[4] = {};
    RigidBody3D* cart = nullptr;
    buildCart(root, wheels, &cart);
    for (int i = 0; i < 4; ++i) wheels[i]->setSpinMotor(10.0f, 400.0f);

    run(tree, 4.0);
    const Math::Vec3 straight = cart->getGlobalPosition();
    printf("  cart straight: x %.2f z %.2f y %.2f heading %.1f speed %.2f\n", straight.x,
            straight.z, straight.y, headingOf(cart), cart->getLinearVelocity().x);
    CHECK(straight.x > 12.0f);
    CHECK(fabsf(straight.z) < 0.5f);
    CHECK(fabsf(headingOf(cart)) < 3.0f);
    CHECK(straight.y > 0.8f && straight.y < 1.2f);

    for (int i = 0; i < 4; ++i)
    {
        wheels[i]->setSteeringServo(0.0f, 200.0f, 3.0f);
        if (i & 1) wheels[i]->setSteeringServo(-0.2f, 200.0f, 3.0f);
    }
    run(tree, 3.0);
    const Math::Vec3 turned = cart->getGlobalPosition();
    printf("  cart steering: x %.2f z %.2f y %.2f heading %.1f, front %.3f %.3f rear %.3f %.3f\n",
            turned.x, turned.z, turned.y, headingOf(cart), wheels[1]->getSteeringAngle(),
            wheels[3]->getSteeringAngle(), wheels[0]->getSteeringAngle(),
            wheels[2]->getSteeringAngle());
    CHECK(headingOf(cart) > 5.0f);
    CHECK(turned.y < 1.3f);
    CHECK(turned.z < -0.5f);
    CHECK(fabsf(wheels[0]->getSteeringAngle()) < 0.02f);
    CHECK(fabsf(wheels[2]->getSteeringAngle()) < 0.02f);
    CHECK(near(wheels[1]->getSteeringAngle(), -0.2f, 0.03f));

    tree.finalize();
}

void testLifecycle()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* frame = root->createChild<StaticBody3D>();
    frame->setName("Frame");
    frame->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    RigidBody3D* bob = root->createChild<RigidBody3D>();
    bob->setName("Bob");
    bob->setPosition(Math::Vec3(1.0f, 5.0f, 0.0f));

    PinJoint3D* pin = root->createChild<PinJoint3D>();
    pin->setPosition(Math::Vec3(0.5f, 5.0f, 0.0f));
    pin->setNodeA("../Frame");
    pin->setNodeB("../Bob");
    tree.physicsProcess(kStep);
    CHECK(!pin->isConfigured());
    CHECK(pin->getConfigurationWarning().empty());

    CollisionShape3D* frameShape = frame->createChild<CollisionShape3D>();
    frameShape->setBox(Math::Vec3(0.5f, 0.5f, 0.5f));
    CollisionShape3D* bobShape = bob->createChild<CollisionShape3D>();
    bobShape->setBox(Math::Vec3(0.5f, 0.5f, 0.5f));
    tree.physicsProcess(kStep);
    tree.physicsProcess(kStep);
    CHECK(pin->isConfigured());
    CHECK(pin->getBodyA() == frame && pin->getBodyB() == bob);
    CHECK(server.bodyHasCollisionException(frame->getBody(), bob->getBody()));

    pin->setExcludeNodesFromCollision(false);
    CHECK(pin->isConfigured());
    CHECK(!server.bodyHasCollisionException(frame->getBody(), bob->getBody()));

    pin->setNodeB("../Missing");
    CHECK(!pin->isConfigured());
    CHECK(pin->getConfigurationWarning().length() > 0);
    pin->setNodeB("../Bob");
    CHECK(pin->isConfigured());

    bob->queueFree();
    tree.physicsProcess(kStep);
    tree.physicsProcess(kStep);
    CHECK(!pin->isConfigured());
    CHECK(server.getStats().joints == 0);

    HingeJoint3D* single = root->createChild<HingeJoint3D>();
    single->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    single->setNodeB("../Frame");
    tree.physicsProcess(kStep);
    CHECK(single->isConfigured());
    CHECK(single->getBodyA() == frame && single->getBodyB() == nullptr);

    single->queueFree();
    tree.physicsProcess(kStep);
    tree.physicsProcess(kStep);
    CHECK(server.getStats().joints == 0);

    tree.finalize();
}

void testConeTwist()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* frame = makeFrame(root, "cone_frame", Math::Vec3(0.0f, 5.0f, 0.0f));
    const Math::Vec3 anchor(0.0f, 5.0f, 0.0f);
    RigidBody3D* bob =
            makeBox(root, "cone_bob", Math::Vec3(0.0f, 3.0f, 0.0f), Math::Vec3(0.2f, 0.2f, 0.2f));
    ConeTwistJoint3D* cone = root->createChild<ConeTwistJoint3D>();
    cone->setPosition(anchor);
    cone->setRotation(Math::Vec3(kPi * 0.5f, 0.0f, 0.0f));
    cone->setSwingLimits(kPi / 6.0f, kPi / 6.0f);
    cone->setTwistLimits(-kPi / 18.0f, kPi / 18.0f);
    cone->setBodies(frame, bob);

    StaticBody3D* frame2 = makeFrame(root, "twist_frame", Math::Vec3(5.0f, 5.0f, 0.0f));
    RigidBody3D* spinner =
            makeBox(root, "twist_bob", Math::Vec3(5.0f, 3.0f, 0.0f), Math::Vec3(0.3f, 0.2f, 0.3f));
    ConeTwistJoint3D* twist = root->createChild<ConeTwistJoint3D>();
    twist->setPosition(Math::Vec3(5.0f, 5.0f, 0.0f));
    twist->setRotation(Math::Vec3(kPi * 0.5f, 0.0f, 0.0f));
    twist->setSwingLimits(kPi / 6.0f, kPi / 6.0f);
    twist->setTwistLimits(-kPi / 18.0f, kPi / 18.0f);
    twist->setBodies(frame2, spinner);

    run(tree, 0.1);
    CHECK(cone->isConfigured());
    CHECK(twist->isConfigured());
    CHECK(server.jointGetType(cone->getJoint()) == PhysicsJointType::ConeTwist);
    spinner->setAngularVelocity(Math::Vec3(0.0f, 6.0f, 0.0f));
    bob->setLinearVelocity(Math::Vec3(3.0f, 0.0f, 0.0f));

    float worstSwing = 0.0f;
    float worstLength = 0.0f;
    float worstYaw = 0.0f;
    for (int i = 0; i < 180; ++i)
    {
        tree.physicsProcess(kStep);
        const Math::Vec3 hang = bob->getGlobalPosition() - anchor;
        const float length = hang.Length();
        const float swing = acosf(-hang.y / length);
        if (swing > worstSwing) worstSwing = swing;
        if (fabsf(length - 2.0f) > worstLength) worstLength = fabsf(length - 2.0f);
        const Math::Mat4 frameOf = spinner->getGlobalTransform();
        const float yaw = fabsf(atan2f(frameOf.col0.z, frameOf.col0.x));
        if (yaw > worstYaw) worstYaw = yaw;
    }
    printf("  cone twist: swing stayed below %.1f deg (limit 30), rope %.3f off 2.000, twist below "
           "%.1f deg (limit 10)\n",
            worstSwing * 180.0f / kPi, worstLength, worstYaw * 180.0f / kPi);
    CHECK(worstSwing < kPi / 6.0f + 0.1f);
    CHECK(worstSwing > kPi / 9.0f);
    CHECK(worstLength < 0.03f);
    CHECK(worstYaw < kPi / 18.0f + 0.06f);
    CHECK(worstYaw > kPi / 36.0f);
    tree.finalize();
}

void testJointedBodiesSleepTogether()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* groundShape = ground->createChild<CollisionShape3D>();
    groundShape->setBox(Math::Vec3(20.0f, 0.5f, 20.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    StaticBody3D* pedestal = root->createChild<StaticBody3D>();
    CollisionShape3D* pedestalShape = pedestal->createChild<CollisionShape3D>();
    pedestalShape->setBox(Math::Vec3(1.0f, 2.0f, 1.0f));
    pedestal->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));

    RigidBody3D* heavy = root->createChild<RigidBody3D>();
    CollisionShape3D* heavyShape = heavy->createChild<CollisionShape3D>();
    heavyShape->setBox(Math::Vec3(0.8f, 0.8f, 0.8f));
    heavy->setPosition(Math::Vec3(0.0f, 2.4f, 0.0f));
    heavy->setMass(20.0f);
    RigidBody3D* light = root->createChild<RigidBody3D>();
    CollisionShape3D* lightShape = light->createChild<CollisionShape3D>();
    lightShape->setBox(Math::Vec3(0.3f, 0.3f, 0.3f));
    light->setPosition(Math::Vec3(1.2f, 2.4f, 0.0f));
    light->setMass(1.0f);
    const Math::Vec3 pivot(0.5f, 2.4f, 0.0f);
    PinJoint3D* pin = root->createChild<PinJoint3D>();
    pin->setPosition(pivot);
    pin->setBodies(heavy, light);

    const Math::Vec3 localA = pivot - heavy->getGlobalPosition();
    const Math::Vec3 localB = pivot - light->getGlobalPosition();
    int mismatched = 0;
    int asleepTogether = 0;
    float worstApart = 0.0f;
    for (int i = 0; i < 360; ++i)
    {
        tree.physicsProcess(kStep);
        const bool heavySleeps = server.bodyIsSleeping(heavy->getBody());
        const bool lightSleeps = server.bodyIsSleeping(light->getBody());
        if (heavySleeps != lightSleeps) ++mismatched;
        if (heavySleeps && lightSleeps) ++asleepTogether;
        const Math::Mat4 a = heavy->getGlobalTransform();
        const Math::Mat4 b = light->getGlobalTransform();
        const Math::Vec3 anchorA = Math::Vec3(a.col3.x, a.col3.y, a.col3.z) +
                                   Math::Vec3(a.col0.x, a.col0.y, a.col0.z) * localA.x +
                                   Math::Vec3(a.col1.x, a.col1.y, a.col1.z) * localA.y +
                                   Math::Vec3(a.col2.x, a.col2.y, a.col2.z) * localA.z;
        const Math::Vec3 anchorB = Math::Vec3(b.col3.x, b.col3.y, b.col3.z) +
                                   Math::Vec3(b.col0.x, b.col0.y, b.col0.z) * localB.x +
                                   Math::Vec3(b.col1.x, b.col1.y, b.col1.z) * localB.y +
                                   Math::Vec3(b.col2.x, b.col2.y, b.col2.z) * localB.z;
        const float gap = (anchorA - anchorB).Length();
        if (gap > worstApart) worstApart = gap;
    }
    printf("  jointed sleep: %d steps with one body asleep and the other awake, %d asleep "
           "together, pin stretched at most %.3f\n",
            mismatched, asleepTogether, worstApart);
    CHECK(mismatched == 0);
    CHECK(worstApart < 0.02f);
    CHECK(heavy->getGlobalPosition().y > 2.0f);
    tree.finalize();
}

} // namespace

int main()
{
    testConeTwist();
    testJointedBodiesSleepTogether();
    testHingeDoor();
    testSliderAndMotor();
    testPinPendulum();
    testFixedAndDistance();
    testPistonUniversalWheelMouse();
    testCart();
    testLifecycle();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
