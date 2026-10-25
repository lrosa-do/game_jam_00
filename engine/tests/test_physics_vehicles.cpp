#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Vehicle3D.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

bool near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

void run(SceneTree& tree, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i) tree.physicsProcess(kStep);
}

StaticBody3D* makeGround(Node* root)
{
    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* shape = ground->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(200.0f, 1.0f, 200.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    return ground;
}

VehicleWheel3D* addWheel(VehicleBody3D* car, const Math::Vec3& position, bool front, bool drive)
{
    VehicleWheel3D* wheel = car->createChild<VehicleWheel3D>();
    wheel->setPosition(position);
    wheel->setRadius(0.4f);
    wheel->setSuspensionRestLength(0.4f);
    wheel->setSuspensionTravel(0.5f);
    wheel->setUseAsSteering(front);
    wheel->setUseAsTraction(drive);
    return wheel;
}

VehicleBody3D* makeCar(Node* root, const Math::Vec3& position)
{
    VehicleBody3D* car = root->createChild<VehicleBody3D>();
    CollisionShape3D* shape = car->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(2.0f, 0.6f, 4.0f));
    car->setPosition(position);
    car->setMass(800.0f);
    car->setFriction(0.8f);
    addWheel(car, Math::Vec3(0.9f, -0.2f, -1.4f), true, false);
    addWheel(car, Math::Vec3(-0.9f, -0.2f, -1.4f), true, false);
    addWheel(car, Math::Vec3(0.9f, -0.2f, 1.4f), false, true);
    addWheel(car, Math::Vec3(-0.9f, -0.2f, 1.4f), false, true);
    return car;
}

void testCar()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);
    VehicleBody3D* car = makeCar(root, Math::Vec3(0.0f, 1.2f, 0.0f));

    run(tree, 1.5);
    CHECK(car->getWheelCount() == 4);
    CHECK(server.vehicleValid(car->getVehicle()));
    CHECK(server.vehicleGetWheelCount(car->getVehicle()) == 4);
    const float rest = car->getGlobalPosition().y;
    printf("  car: rest height %.3f, wheel 0 in contact %d, skid %.2f\n", rest,
            car->getWheel(0)->isInContact() ? 1 : 0, car->getWheel(0)->getSkidInfo());
    CHECK(rest > 0.3f && rest < 1.1f);
    CHECK(car->getWheel(0)->isInContact());
    CHECK(car->getWheel(0)->getContactBody() != nullptr);
    CHECK(near(car->getWheel(0)->getContactNormal().y, 1.0f, 0.1f));
    const Math::Mat4 wheelFrame = car->getWheel(0)->getTransform();
    printf("  car: wheel 0 local (%.3f %.3f %.3f)\n", wheelFrame.col3.x, wheelFrame.col3.y,
            wheelFrame.col3.z);
    CHECK(near(wheelFrame.col3.x, 0.9f, 0.05f));
    CHECK(near(wheelFrame.col3.z, -1.4f, 0.05f));
    CHECK(wheelFrame.col3.y < -0.1f);

    car->setEngineForce(900.0f);
    run(tree, 3.0);
    printf("  car: z %.2f, speed %.1f km/h, rear wheel rpm %.1f\n", car->getGlobalPosition().z,
            car->getSpeedKmh(), car->getWheel(2)->getRpm());
    CHECK(car->getGlobalPosition().z < -5.0f);
    CHECK(fabsf(car->getSpeedKmh()) > 10.0f);
    CHECK(fabsf(car->getWheel(2)->getRpm()) > 5.0f);
    CHECK(near(car->getWheel(0)->getEngineForce(), 0.0f, 1e-3f));
    CHECK(near(car->getWheel(2)->getEngineForce(), 900.0f, 1e-3f));

    car->setSteering(0.4f);
    CHECK(near(car->getWheel(0)->getSteering(), 0.4f, 1e-4f));
    CHECK(near(car->getWheel(2)->getSteering(), 0.0f, 1e-4f));
    run(tree, 3.0);
    CHECK(car->getGlobalPosition().x < -1.0f);

    car->setEngineForce(0.0f);
    car->setSteering(0.0f);
    car->setBrake(20.0f);
    run(tree, 8.0);
    printf("  car: speed after braking %.2f km/h\n", car->getSpeedKmh());
    CHECK(fabsf(car->getSpeedKmh()) < 1.0f);

    car->getWheel(0)->setSuspensionStiffness(20.0f);
    CHECK(near(car->getWheel(0)->getSuspensionStiffness(), 20.0f, 1e-4f));

    tree.finalize();
}

void testWheelLifecycle()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);
    VehicleBody3D* car = makeCar(root, Math::Vec3(0.0f, 1.2f, 0.0f));
    run(tree, 0.5);
    CHECK(server.vehicleGetWheelCount(car->getVehicle()) == 4);

    car->getWheel(3)->queueFree();
    run(tree, 0.2);
    CHECK(car->getWheelCount() == 3);
    CHECK(server.vehicleGetWheelCount(car->getVehicle()) == 3);

    VehicleWheel3D* extra = addWheel(car, Math::Vec3(0.0f, -0.2f, 0.0f), false, false);
    run(tree, 0.2);
    CHECK(car->getWheelCount() == 4);
    CHECK(server.vehicleGetWheelCount(car->getVehicle()) == 4);
    CHECK(extra->isInsideTree());

    car->queueFree();
    run(tree, 0.2);
    CHECK(server.getStats().vehicles == 0);
    tree.finalize();
}

void testMotorcycleNode()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    VehicleBody3D* bike = root->createChild<VehicleBody3D>();
    CollisionShape3D* shape = bike->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(0.4f, 0.6f, 1.8f));
    bike->setPosition(Math::Vec3(0.0f, 1.3f, 0.0f));
    bike->setMass(200.0f);
    bike->setInertia(Math::Vec3(60.0f, 60.0f, 40.0f));
    bike->setFriction(0.05f);
    bike->setCanSleep(false);
    VehicleWheel3D* front = bike->createChild<VehicleWheel3D>();
    front->setPosition(Math::Vec3(0.0f, -0.1f, -0.8f));
    front->setRadius(0.35f);
    front->setSuspensionRestLength(0.45f);
    front->setSuspensionTravel(0.6f);
    front->setUseAsSteering(true);
    VehicleWheel3D* rear = bike->createChild<VehicleWheel3D>();
    rear->setPosition(Math::Vec3(0.0f, -0.1f, 0.8f));
    rear->setRadius(0.35f);
    rear->setSuspensionRestLength(0.45f);
    rear->setSuspensionTravel(0.6f);
    rear->setUseAsTraction(true);
    bike->setMotorcycle(true);
    CHECK(bike->isMotorcycle());

    run(tree, 1.0);
    CHECK(server.vehicleIsMotorcycle(bike->getVehicle()));
    bike->applyTorqueImpulse(Math::Vec3(0.0f, 0.0f, 1.5f));
    run(tree, 3.0);
    const Math::Mat4 pose = bike->getGlobalTransform();
    printf("  motorcycle node: up.y %.3f, lean %.3f\n", pose.col1.y, bike->getLeanAngle());
    CHECK(pose.col1.y > 0.95f);

    bike->setEngineForce(700.0f);
    bike->setSteerInput(0.5f);
    run(tree, 3.0);
    printf("  motorcycle node: speed %.1f km/h, x %.2f, up.y %.3f\n", bike->getSpeedKmh(),
            bike->getGlobalPosition().x, bike->getGlobalTransform().col1.y);
    CHECK(fabsf(bike->getSpeedKmh()) > 5.0f);
    CHECK(fabsf(bike->getGlobalPosition().x) > 0.3f);
    tree.finalize();
}

} // namespace

int main()
{
    testCar();
    testWheelLifecycle();
    testMotorcycleNode();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
