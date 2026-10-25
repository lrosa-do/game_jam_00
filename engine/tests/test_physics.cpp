#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/Batch.h"

#include <malloc.h>
#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

Math::Mat4 at(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

PhysicsBodyId makeBody(PhysicsServer3D& server, PhysicsShapeId shape, PhysicsBodyMode mode,
        const Math::Mat4& frame, float friction = 0.55f)
{
    const PhysicsBodyId body = server.bodyCreate();
    server.bodySetMode(body, mode);
    server.bodySetShape(body, shape, Math::Mat4::Identity());
    server.bodySetFriction(body, friction);
    server.bodySetTransform(body, frame);
    return body;
}

PhysicsShapeId boxShape(PhysicsServer3D& server, const Math::Vec3& half)
{
    const PhysicsShapeId shape = server.shapeCreate(PhysicsShapeType::Box);
    server.shapeSetBox(shape, half);
    return shape;
}

PhysicsShapeId sphereShape(PhysicsServer3D& server, float radius)
{
    const PhysicsShapeId shape = server.shapeCreate(PhysicsShapeType::Sphere);
    server.shapeSetSphere(shape, radius);
    return shape;
}

PhysicsBodyId makeGround(PhysicsServer3D& server, float friction = 0.55f)
{
    return makeBody(server, boxShape(server, Math::Vec3(100.0f, 1.0f, 100.0f)),
            PhysicsBodyMode::Static, at(0.0f, -1.0f, 0.0f), friction);
}

float yOf(const PhysicsServer3D& server, PhysicsBodyId body)
{
    return server.bodyGetTransform(body).col3.y;
}

void run(PhysicsServer3D& server, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i) server.step(kStep);
}

void testFall()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    makeGround(server);
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 10.0f, 0.0f));

    CHECK(near(yOf(server, ball), 10.0f));
    int hitStep = -1;
    bool fast = false;
    for (int i = 1; i <= 240; ++i)
    {
        server.step(kStep);
        const float vy = server.bodyGetLinearVelocity(ball).y;
        if (vy < -10.0f) fast = true;
        if (fast && hitStep < 0 && vy > -6.8f) hitStep = i;
    }
    const double analytic = sqrt(2.0 * 9.5 / 9.8);
    const double measured = hitStep * kStep;
    printf("  fall: analytic %.4f s, impact step %d (%.4f s)\n", analytic, hitStep, measured);
    CHECK(hitStep > 0);
    CHECK(fabs(measured - analytic) <= 2.0 * kStep);
    CHECK(near(yOf(server, ball), 0.5f, 0.02f));
    CHECK(server.getStats().steps == 240);
}

ct::Vector<Math::Vec3> cubeCorners(float half, const Math::Vec3& offset)
{
    ct::Vector<Math::Vec3> corners;
    for (int i = 0; i < 8; ++i)
        corners.push_back(Math::Vec3(((i & 1) ? half : -half) + offset.x,
                ((i & 2) ? half : -half) + offset.y, ((i & 4) ? half : -half) + offset.z));
    return corners;
}

PhysicsShapeId hullShape(PhysicsServer3D& server, const ct::Vector<Math::Vec3>& points)
{
    const PhysicsShapeId shape = server.shapeCreate(PhysicsShapeType::ConvexHull);
    server.shapeSetConvexHull(shape, points.data(), points.size());
    return shape;
}

void testConvexHull()
{
    {
        PhysicsServer3D server;
        const PhysicsShapeId shape =
                hullShape(server, cubeCorners(0.5f, Math::Vec3(3.0f, 0.0f, 0.0f)));
        CHECK(server.shapeGetType(shape) == PhysicsShapeType::ConvexHull);
        CHECK(server.shapeGetConvexVertexCount(shape) == 8);
        const Math::Vec3 center = server.shapeGetConvexCenter(shape);
        CHECK(near(center.x, 3.0f, 1e-4f) && near(center.y, 0.0f, 1e-4f) &&
                near(center.z, 0.0f, 1e-4f));
        const PhysicsShapeId flat =
                hullShape(server, cubeCorners(0.5f, Math::Vec3(0.0f, 0.0f, 0.0f)));
        ct::Vector<Math::Vec3> coplanar;
        coplanar.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
        coplanar.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
        coplanar.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        coplanar.push_back(Math::Vec3(1.0f, 0.0f, 1.0f));
        const PhysicsShapeId degenerate = hullShape(server, coplanar);
        CHECK(server.shapeGetConvexVertexCount(flat) == 8);
        CHECK(server.shapeGetConvexVertexCount(degenerate) == 0);
    }

    for (int variant = 0; variant < 2; ++variant)
    {
        PhysicsServer3D server;
        server.setDefaultLinearDamp(0.0f);
        server.setDefaultAngularDamp(0.0f);
        makeGround(server);
        const PhysicsShapeId shape =
                hullShape(server, cubeCorners(0.5f, Math::Vec3(3.0f, 0.0f, 0.0f)));
        const float scale = variant == 0 ? 1.0f : 2.0f;
        const PhysicsBodyId body = server.bodyCreate();
        server.bodySetMode(body, PhysicsBodyMode::Rigid);
        server.bodySetShape(body, shape, Math::Mat4::Scale(Math::Vec3(scale, scale, scale)));
        server.bodySetFriction(body, 0.55f);
        server.bodySetTransform(body, at(0.0f, 5.0f, 0.0f));
        CHECK(server.bodyIsSimulated(body));
        run(server, 3.0);
        const Math::Mat4 frame = server.bodyGetTransform(body);
        printf("  hull scale %.0f: node y %.3f x %.3f z %.3f\n", scale, frame.col3.y, frame.col3.x,
                frame.col3.z);
        CHECK(near(frame.col3.y, 0.5f * scale, 0.03f));
        CHECK(near(frame.col3.x, 0.0f, 0.05f));
        CHECK(near(frame.col3.z, 0.0f, 0.05f));
        CHECK(server.bodyIsSleeping(body));
    }

    {
        PhysicsServer3D server;
        makeGround(server);
        const PhysicsShapeId shape =
                hullShape(server, cubeCorners(0.5f, Math::Vec3(0.0f, 0.0f, 0.0f)));
        const PhysicsBodyId first =
                makeBody(server, shape, PhysicsBodyMode::Static, at(-5.0f, 0.5f, 0.0f));
        const PhysicsBodyId second =
                makeBody(server, shape, PhysicsBodyMode::Static, at(5.0f, 0.5f, 0.0f));
        PhysicsRayParameters ray;
        ray.from = Math::Vec3(-5.0f, 10.0f, 0.0f);
        ray.to = Math::Vec3(-5.0f, -1.0f, 0.0f);
        PhysicsRayResult hit;
        CHECK(server.intersectRay(ray, hit));
        CHECK(hit.body == first);
        CHECK(near(hit.position.y, 1.0f, 1e-3f));
        ray.from = Math::Vec3(5.0f, 10.0f, 0.0f);
        ray.to = Math::Vec3(5.0f, -1.0f, 0.0f);
        CHECK(server.intersectRay(ray, hit));
        CHECK(hit.body == second);

        server.shapeSetConvexHull(shape, cubeCorners(1.5f, Math::Vec3(0.0f, 0.0f, 0.0f)).data(), 8);
        ray.from = Math::Vec3(-5.0f, 10.0f, 0.0f);
        ray.to = Math::Vec3(-5.0f, -1.0f, 0.0f);
        CHECK(server.intersectRay(ray, hit));
        CHECK(near(hit.position.y, 2.0f, 1e-3f));
        ray.from = Math::Vec3(5.0f, 10.0f, 0.0f);
        ray.to = Math::Vec3(5.0f, -1.0f, 0.0f);
        CHECK(server.intersectRay(ray, hit));
        CHECK(near(hit.position.y, 2.0f, 1e-3f));
    }
}

void testConvexHullOnMesh()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsShapeId floorShape = server.shapeCreate(PhysicsShapeType::ConcavePolygon);
    const Math::Vec3 vertices[4] = { Math::Vec3(-10.0f, 0.0f, -10.0f),
        Math::Vec3(-10.0f, 0.0f, 10.0f), Math::Vec3(10.0f, 0.0f, 10.0f),
        Math::Vec3(10.0f, 0.0f, -10.0f) };
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    server.shapeSetConcaveFaces(floorShape, vertices, 4, indices, 6);
    makeBody(server, floorShape, PhysicsBodyMode::Static, at(0.0f, 0.0f, 0.0f));

    const PhysicsShapeId cube = hullShape(server, cubeCorners(0.5f, Math::Vec3(0.0f, 0.0f, 0.0f)));
    const PhysicsBodyId flat = makeBody(server, cube, PhysicsBodyMode::Rigid, at(0.0f, 3.0f, 0.0f));
    const Math::Quaternion tilt =
            Math::Quaternion::FromAxisAngle(Math::Vec3(1.0f, 0.0f, 1.0f).Normalized(), 0.4f);
    const PhysicsBodyId tilted = makeBody(server, cube, PhysicsBodyMode::Rigid,
            Math::Mat4(tilt.ToMat3(), Math::Vec3(4.0f, 3.0f, 0.0f)));
    const PhysicsShapeId box = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    const PhysicsBodyId reference =
            makeBody(server, box, PhysicsBodyMode::Rigid, at(-4.0f, 3.0f, 0.0f));
    run(server, 4.0);
    printf("  hull on mesh: flat y %.3f, tilted y %.3f, box y %.3f\n", yOf(server, flat),
            yOf(server, tilted), yOf(server, reference));
    CHECK(near(yOf(server, flat), 0.5f, 0.02f));
    CHECK(near(yOf(server, tilted), 0.5f, 0.02f));
    CHECK(near(yOf(server, reference), 0.5f, 0.02f));
    CHECK(server.bodyIsSleeping(flat));
    CHECK(server.bodyIsSleeping(tilted));

    const PhysicsBodyId slab = makeBody(server, boxShape(server, Math::Vec3(10.0f, 0.5f, 10.0f)),
            PhysicsBodyMode::Static, at(30.0f, -0.5f, 0.0f));
    (void) slab;
    const PhysicsBodyId onBox =
            makeBody(server, cube, PhysicsBodyMode::Rigid, at(30.0f, 3.0f, 0.0f));
    run(server, 3.0);
    CHECK(near(yOf(server, onBox), 0.5f, 0.02f));
    CHECK(server.bodyIsSleeping(onBox));
}

Math::Vec3 positionOf(PhysicsServer3D& server, PhysicsBodyId body)
{
    const Math::Mat4 frame = server.bodyGetTransform(body);
    return Math::Vec3(frame.col3.x, frame.col3.y, frame.col3.z);
}

float distanceBetween(PhysicsServer3D& server, PhysicsBodyId a, PhysicsBodyId b)
{
    return (positionOf(server, a) - positionOf(server, b)).Length();
}

PhysicsBodyId zeroGravityBody(PhysicsServer3D& server, PhysicsBodyMode mode, const Math::Vec3& at_)
{
    return makeBody(server, boxShape(server, Math::Vec3(0.25f, 0.25f, 0.25f)), mode,
            at(at_.x, at_.y, at_.z), 0.5f);
}

void testJointDistance()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId anchor =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 10.0f, 0.0f));
    const PhysicsBodyId bob =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(2.0f, 10.0f, 0.0f));
    PhysicsJointFrame frame;
    frame.anchor = Math::Vec3(0.0f, 10.0f, 0.0f);
    frame.anchorB = Math::Vec3(2.0f, 10.0f, 0.0f);
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Distance, anchor, bob, frame);
    CHECK(server.jointValid(joint));
    CHECK(server.jointGetType(joint) == PhysicsJointType::Distance);
    server.jointSetDistance(joint, 2.0f, 2.0f);
    float low = 1e9f;
    float high = 0.0f;
    for (int i = 0; i < 240; ++i)
    {
        server.step(kStep);
        const float d = distanceBetween(server, anchor, bob);
        if (d < low) low = d;
        if (d > high) high = d;
    }
    printf("  distance joint: %.3f to %.3f, bob y %.2f\n", low, high, yOf(server, bob));
    CHECK(low > 1.9f && high < 2.1f);
    CHECK(yOf(server, bob) < 10.0f);
}

void testJointFixed()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId anchor =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 10.0f, 0.0f));
    const PhysicsBodyId first =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(1.0f, 10.0f, 0.0f));
    const PhysicsBodyId second =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(2.0f, 10.0f, 0.0f));
    server.jointCreate(PhysicsJointType::Fixed, anchor, first, Math::Vec3(0.5f, 10.0f, 0.0f),
            Math::Vec3(0.0f, 1.0f, 0.0f));
    server.jointCreate(PhysicsJointType::Fixed, first, second, Math::Vec3(1.5f, 10.0f, 0.0f),
            Math::Vec3(0.0f, 1.0f, 0.0f));
    run(server, 3.0);
    const Math::Vec3 a = positionOf(server, first);
    const Math::Vec3 b = positionOf(server, second);
    printf("  fixed joints: first (%.3f %.3f), second (%.3f %.3f)\n", a.x, a.y, b.x, b.y);
    CHECK(near(a.x, 1.0f, 0.05f) && near(a.y, 10.0f, 0.05f));
    CHECK(near(b.x, 2.0f, 0.08f) && near(b.y, 10.0f, 0.08f));
}

void testJointSlider()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId rail =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsBodyId car =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(0.0f, 5.0f, 0.0f));
    server.bodySetCollisionLayer(rail, 0);
    server.bodySetCollisionMask(rail, 0);
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Slider, rail, car,
            Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f));
    server.jointSetLimits(joint, -1.0f, 3.0f);
    server.bodySetLinearVelocity(car, Math::Vec3(4.0f, 2.0f, 1.0f));
    run(server, 2.0);
    const Math::Vec3 p = positionOf(server, car);
    printf("  slider: x %.3f y %.3f z %.3f, position %.3f\n", p.x, p.y, p.z,
            server.jointGetPosition(joint));
    CHECK(near(p.x, 3.0f, 0.1f));
    CHECK(near(p.y, 5.0f, 0.05f) && near(p.z, 0.0f, 0.05f));
    CHECK(near(server.jointGetPosition(joint), 3.0f, 0.1f));

    server.jointSetLimits(joint, -100.0f, 100.0f);
    server.jointSetMotor(joint, -2.0f, 1000.0f);
    run(server, 2.0);
    printf("  slider motor: position %.3f\n", server.jointGetPosition(joint));
    CHECK(server.jointGetPosition(joint) < 0.0f);
}

void testJointPiston()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId base =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsBodyId rod =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Piston, base, rod,
            Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    server.jointSetLimits(joint, -1.0f, 2.0f);
    server.jointSetMotor(joint, 3.0f, 1000.0f);
    server.jointSetSecondaryMotor(joint, 2.0f, 1000.0f);
    run(server, 2.0);
    printf("  piston: position %.3f angle %.3f, rod (%.2f %.2f %.2f)\n",
            server.jointGetPosition(joint), server.jointGetSecondaryPosition(joint),
            positionOf(server, rod).x, positionOf(server, rod).y, positionOf(server, rod).z);
    CHECK(near(server.jointGetPosition(joint), 2.0f, 0.1f));
    CHECK(fabsf(server.jointGetSecondaryPosition(joint)) > 0.5f);
    CHECK(near(positionOf(server, rod).x, 0.0f, 0.05f));
    server.jointDisableMotor(joint);
    server.jointDisableSecondaryMotor(joint);
}

void testJointUniversal()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId base =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsBodyId arm =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(0.0f, 5.0f, 1.0f));
    PhysicsJointFrame frame;
    frame.anchor = Math::Vec3(0.0f, 5.0f, 0.0f);
    frame.axis = Math::Vec3(1.0f, 0.0f, 0.0f);
    frame.axisB = Math::Vec3(0.0f, 1.0f, 0.0f);
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Universal, base, arm, frame);
    CHECK(server.jointValid(joint));
    server.jointSetMotor(joint, 1.0f, 100.0f);
    run(server, 1.0);
    printf("  universal: angle A %.3f, angle B %.3f\n", server.jointGetPosition(joint),
            server.jointGetSecondaryPosition(joint));
    CHECK(fabsf(server.jointGetPosition(joint)) > 0.4f);
    CHECK(fabsf(server.jointGetSecondaryPosition(joint)) < 0.1f);
    CHECK(distanceBetween(server, base, arm) < 1.05f && distanceBetween(server, base, arm) > 0.95f);
}

void testJointWheel()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId chassis =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    server.bodySetCollisionLayer(chassis, 0);
    server.bodySetCollisionMask(chassis, 0);
    const PhysicsBodyId wheel = makeBody(server, sphereShape(server, 0.4f), PhysicsBodyMode::Rigid,
            at(0.0f, 4.0f, 0.0f));
    server.bodySetMass(wheel, 15.0f);
    PhysicsJointFrame frame;
    frame.anchor = Math::Vec3(0.0f, 4.0f, 0.0f);
    frame.axis = Math::Vec3(0.0f, -1.0f, 0.0f);
    frame.axisB = Math::Vec3(1.0f, 0.0f, 0.0f);
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Wheel, chassis, wheel, frame);
    CHECK(server.jointValid(joint));
    server.jointSetSuspension(joint, 0.0f, 2000.0f, 200.0f);

    server.jointSetSteeringServo(joint, 0.3f, 500.0f, 5.0f);
    run(server, 1.0);
    CHECK(near(server.jointGetSecondaryPosition(joint), 0.3f, 0.03f));

    server.jointSetMotor(joint, 6.0f, 500.0f);
    run(server, 3.0);
    const Math::Vec3 w = server.bodyGetAngularVelocity(wheel);
    const Math::Mat4 pose = server.bodyGetTransform(wheel);
    printf("  wheel: spin %.2f rad/s, steering %.3f, axle y %.4f, travel %.3f\n", w.Length(),
            server.jointGetSecondaryPosition(joint), pose.col0.y, server.jointGetPosition(joint));
    CHECK(near(w.Length(), 6.0f, 0.3f));
    CHECK(near(server.jointGetSecondaryPosition(joint), 0.3f, 0.05f));
    CHECK(fabsf(pose.col0.y) < 0.02f);
    CHECK(fabsf(server.jointGetPosition(joint)) < 0.05f);

    server.jointDisableSteeringServo(joint);
    server.jointDisableMotor(joint);
}

void testJointMouse()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.5f);
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 2.0f, 0.0f));
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Mouse, ball, PhysicsBodyId(),
            Math::Vec3(0.0f, 2.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(server.jointValid(joint));
    server.jointSetMouseSpring(joint, 2000.0f, 5.0f, 1.0f);
    server.jointSetTarget(joint, Math::Vec3(6.0f, 4.0f, 0.0f));
    run(server, 3.0);
    const Math::Vec3 p = positionOf(server, ball);
    printf("  mouse: ball at %.2f %.2f %.2f\n", p.x, p.y, p.z);
    CHECK((p - Math::Vec3(6.0f, 4.0f, 0.0f)).Length() < 0.5f);
}

void testJointPinMotor()
{
    PhysicsServer3D server;
    server.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    const PhysicsBodyId base =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsBodyId spinner =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Pin, base, spinner,
            Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    server.jointSetPinMotor(joint, Math::Vec3(0.0f, 1.0f, 0.0f), 3.0f, 100.0f);
    run(server, 1.0);
    const Math::Vec3 w = server.bodyGetAngularVelocity(spinner);
    printf("  pin motor: angular velocity %.2f %.2f %.2f\n", w.x, w.y, w.z);
    CHECK(near(w.y, 3.0f, 0.4f));
    server.jointDisableMotor(joint);
}

void testCollisionExceptions()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    makeGround(server);
    const PhysicsBodyId slab = makeBody(server, boxShape(server, Math::Vec3(2.0f, 0.25f, 2.0f)),
            PhysicsBodyMode::Static, at(0.0f, 2.0f, 0.0f));
    const PhysicsBodyId resting = makeBody(server, sphereShape(server, 0.5f),
            PhysicsBodyMode::Rigid, at(0.0f, 4.0f, 0.0f));
    const PhysicsBodyId passing = makeBody(server, sphereShape(server, 0.5f),
            PhysicsBodyMode::Rigid, at(1.0f, 4.0f, 0.0f));
    server.bodyAddCollisionException(passing, slab);
    CHECK(server.bodyHasCollisionException(slab, passing));
    CHECK(!server.bodyHasCollisionException(slab, resting));
    run(server, 3.0);
    printf("  exceptions: resting y %.3f, passing y %.3f\n", yOf(server, resting),
            yOf(server, passing));
    CHECK(near(yOf(server, resting), 2.75f, 0.05f));
    CHECK(near(yOf(server, passing), 0.5f, 0.05f));

    server.bodyRemoveCollisionException(passing, slab);
    CHECK(!server.bodyHasCollisionException(passing, slab));
    server.bodySetTransform(passing, at(1.0f, 4.0f, 1.2f));
    run(server, 3.0);
    CHECK(near(yOf(server, passing), 2.75f, 0.05f));

    server.bodyAddCollisionException(passing, slab);
    server.bodyFree(slab);
    server.step(kStep);
    CHECK(!server.bodyHasCollisionException(passing, slab));
}

void testJointFreeing()
{
    PhysicsServer3D server;
    const PhysicsBodyId anchor =
            zeroGravityBody(server, PhysicsBodyMode::Static, Math::Vec3(0.0f, 5.0f, 0.0f));
    const PhysicsBodyId bob =
            zeroGravityBody(server, PhysicsBodyMode::Rigid, Math::Vec3(1.0f, 5.0f, 0.0f));
    ct::Vector<PhysicsJointId> joints;
    const PhysicsJointType types[8] = { PhysicsJointType::Pin, PhysicsJointType::Hinge,
        PhysicsJointType::Distance, PhysicsJointType::Fixed, PhysicsJointType::Slider,
        PhysicsJointType::Piston, PhysicsJointType::Universal, PhysicsJointType::Wheel };
    for (int i = 0; i < 8; ++i)
    {
        PhysicsJointFrame frame;
        frame.anchor = Math::Vec3(0.5f, 5.0f, 0.0f);
        frame.anchorB = Math::Vec3(1.0f, 5.0f, 0.0f);
        frame.axis = Math::Vec3(0.0f, 1.0f, 0.0f);
        frame.axisB = Math::Vec3(1.0f, 0.0f, 0.0f);
        const PhysicsJointId id = server.jointCreate(types[i], anchor, bob, frame);
        CHECK(server.jointValid(id));
        CHECK(server.jointGetType(id) == types[i]);
        joints.push_back(id);
    }
    server.step(kStep);
    CHECK(server.getStats().joints == 8);
    for (std::size_t i = 0; i < joints.size(); ++i) server.jointFree(joints[i]);
    server.step(kStep);
    CHECK(server.getStats().joints == 0);
    const PhysicsJointId again = server.jointCreate(PhysicsJointType::Pin, anchor, bob,
            Math::Vec3(0.5f, 5.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(server.jointValid(again));
    server.bodyFree(bob);
    server.step(kStep);
    CHECK(!server.jointValid(again));
    CHECK(server.getStats().joints == 0);
}

void testBounce()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    makeGround(server);
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 5.0f, 0.0f));
    server.bodySetBounce(ball, 0.5f);

    float apex = 0.0f;
    bool bounced = false;
    for (int i = 0; i < 240; ++i)
    {
        server.step(kStep);
        const float y = yOf(server, ball);
        const float vy = server.bodyGetLinearVelocity(ball).y;
        if (vy > 1.0f) bounced = true;
        if (bounced && y > apex) apex = y;
    }
    printf("  bounce: expected apex 1.625, measured %.3f\n", apex);
    CHECK(bounced);
    CHECK(fabsf(apex - 1.625f) < 0.2f);
}

void testSlope()
{
    const float angle = 0.5235988f;
    for (int variant = 0; variant < 2; ++variant)
    {
        const float friction = variant == 0 ? 0.2f : 1.0f;
        PhysicsServer3D server;
        server.setDefaultLinearDamp(0.0f);
        server.setDefaultAngularDamp(0.0f);
        const Math::Quaternion tilt =
                Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), angle);
        const Math::Mat4 rampFrame = Math::Mat4(tilt.ToMat3(), Math::Vec3(0.0f, 0.0f, 0.0f));
        const PhysicsBodyId ramp = makeBody(server, boxShape(server, Math::Vec3(20.0f, 0.5f, 5.0f)),
                PhysicsBodyMode::Static, rampFrame, friction);
        (void) ramp;
        const Math::Vec3 normal = tilt * Math::Vec3(0.0f, 1.0f, 0.0f);
        const Math::Mat4 boxFrame = Math::Mat4(tilt.ToMat3(), normal * 1.0f);
        const PhysicsBodyId box = makeBody(server, boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f)),
                PhysicsBodyMode::Rigid, boxFrame, friction);
        run(server, 1.0);
        const Math::Vec3 downhill = tilt * Math::Vec3(-1.0f, 0.0f, 0.0f);
        const float speed = Math::Vec3::Dot(server.bodyGetLinearVelocity(box), downhill);
        const float expected = 9.8f * (sinf(angle) - friction * cosf(angle));
        if (variant == 0)
        {
            printf("  slope mu 0.2: expected speed after 1 s %.3f, measured %.3f\n", expected,
                    speed);
            CHECK(fabsf(speed - expected) < 0.35f);
        }
        else
        {
            printf("  slope mu 1.0: tan(30) = 0.577 < 1, speed after 1 s %.4f\n", speed);
            CHECK(fabsf(speed) < 0.1f);
        }
    }
}

void stackCase(int count, unsigned velocityIterations, unsigned positionIterations)
{
    PhysicsServer3D server;
    server.setSolverIterations(velocityIterations, positionIterations);
    makeGround(server);
    const PhysicsShapeId shape = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    PhysicsBodyId stack[8];
    for (int i = 0; i < count; ++i)
        stack[i] = makeBody(server, shape, PhysicsBodyMode::Rigid,
                at(0.0f, 0.5f + static_cast<float>(i) * 1.05f, 0.0f));
    run(server, 12.0);
    bool ordered = true;
    for (int i = 0; i < count; ++i)
        if (fabsf(yOf(server, stack[i]) - (0.5f + static_cast<float>(i))) > 0.1f) ordered = false;
    printf("  stack of %d at %u/%u iterations: top y %.3f, awake bodies %u\n", count,
            velocityIterations, positionIterations, yOf(server, stack[count - 1]),
            server.getStats().awakeBodies);
    CHECK(ordered);
    CHECK(server.getStats().awakeBodies == 0);
    for (int i = 0; i < count; ++i) CHECK(server.bodyIsSleeping(stack[i]));
}

void testStack()
{
    stackCase(3, 8, 3);
    stackCase(5, 30, 10);
}

void testRaycast()
{
    PhysicsServer3D server;
    PhysicsRayResult hit;
    PhysicsRayParameters ray;

    makeBody(server, sphereShape(server, 1.0f), PhysicsBodyMode::Static, at(0.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(0.0f, 0.0f, 5.0f);
    ray.to = Math::Vec3(0.0f, 0.0f, -5.0f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 4.0f) && near(hit.normal.z, 1.0f) && near(hit.position.z, 1.0f));

    makeBody(server, boxShape(server, Math::Vec3(1.0f, 2.0f, 3.0f)), PhysicsBodyMode::Static,
            at(20.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(25.0f, 0.5f, 0.5f);
    ray.to = Math::Vec3(15.0f, 0.5f, 0.5f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 4.0f) && near(hit.normal.x, 1.0f));

    const PhysicsShapeId capsule = server.shapeCreate(PhysicsShapeType::Capsule);
    server.shapeSetCapsule(capsule, 0.5f, 2.0f);
    makeBody(server, capsule, PhysicsBodyMode::Static, at(40.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(45.0f, 0.0f, 0.0f);
    ray.to = Math::Vec3(35.0f, 0.0f, 0.0f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 4.5f) && near(hit.normal.x, 1.0f));
    ray.from = Math::Vec3(40.0f, 5.0f, 0.0f);
    ray.to = Math::Vec3(40.0f, -5.0f, 0.0f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 4.0f) && near(hit.normal.y, 1.0f));

    const PhysicsShapeId mesh = server.shapeCreate(PhysicsShapeType::ConcavePolygon);
    const Math::Vec3 quad[4] = { Math::Vec3(-5.0f, 3.0f, -5.0f), Math::Vec3(-5.0f, 3.0f, 5.0f),
        Math::Vec3(5.0f, 3.0f, 5.0f), Math::Vec3(5.0f, 3.0f, -5.0f) };
    const std::uint32_t quadIndices[6] = { 0, 1, 2, 0, 2, 3 };
    server.shapeSetConcaveFaces(mesh, quad, 4, quadIndices, 6);
    makeBody(server, mesh, PhysicsBodyMode::Static, at(60.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(61.0f, 10.0f, 1.0f);
    ray.to = Math::Vec3(61.0f, -10.0f, 1.0f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 7.0f) && near(hit.normal.y, 1.0f));

    const PhysicsShapeId map = server.shapeCreate(PhysicsShapeType::HeightMap);
    const float heights[9] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    server.shapeSetHeightMap(map, 3, 3, heights);
    makeBody(server, map, PhysicsBodyMode::Static, at(80.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(80.3f, 6.0f, -0.4f);
    ray.to = Math::Vec3(80.3f, -6.0f, -0.4f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 5.0f) && near(hit.normal.y, 1.0f));

    ray.from = Math::Vec3(0.0f, 0.0f, 50.0f);
    ray.to = Math::Vec3(0.0f, 0.0f, 40.0f);
    CHECK(!server.intersectRay(ray, hit));

    ct::Vector<PhysicsBodyId> found;
    server.overlapSphere(Math::Vec3(20.0f, 0.0f, 0.0f), 0.5f, 0xFFFFFFFFu, found);
    CHECK(found.size() == 1);
    server.overlapSphere(Math::Vec3(30.0f, 10.0f, 0.0f), 0.5f, 0xFFFFFFFFu, found);
    CHECK(found.empty());
    server.overlapBox(Math::Box(Math::Vec3(19, -1, -1), Math::Vec3(21, 1, 1)), 0xFFFFFFFFu, found);
    CHECK(found.size() == 1);

    PhysicsServer3D planeServer;
    PhysicsRayResult planeHit;
    const PhysicsShapeId plane = planeServer.shapeCreate(PhysicsShapeType::WorldBoundary);
    planeServer.shapeSetWorldBoundary(plane, Math::Vec3(0.0f, 1.0f, 0.0f), 2.0f);
    makeBody(planeServer, plane, PhysicsBodyMode::Static, at(200.0f, 0.0f, 0.0f));
    ray.from = Math::Vec3(200.0f, 10.0f, 0.0f);
    ray.to = Math::Vec3(200.0f, -10.0f, 0.0f);
    CHECK(planeServer.intersectRay(ray, planeHit));
    CHECK(near(planeHit.distance, 8.0f) && near(planeHit.normal.y, 1.0f));
}

void testLayers()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    const PhysicsBodyId ground = makeGround(server);
    server.bodySetCollisionLayer(ground, 1);
    server.bodySetCollisionMask(ground, 3);
    const PhysicsShapeId shape = sphereShape(server, 0.5f);
    const PhysicsBodyId a = makeBody(server, shape, PhysicsBodyMode::Rigid, at(0.0f, 3.0f, 0.0f));
    server.bodySetCollisionLayer(a, 1);
    server.bodySetCollisionMask(a, 1);
    const PhysicsBodyId b = makeBody(server, shape, PhysicsBodyMode::Rigid, at(5.0f, 3.0f, 0.0f));
    server.bodySetCollisionLayer(b, 2);
    server.bodySetCollisionMask(b, 2);
    run(server, 3.0);
    CHECK(near(yOf(server, a), 0.5f, 0.02f));
    CHECK(yOf(server, b) < -10.0f);

    PhysicsRayParameters ray;
    ray.from = Math::Vec3(0.0f, 5.0f, 2.0f);
    ray.to = Math::Vec3(0.0f, -5.0f, 2.0f);
    PhysicsRayResult hit;
    ray.collisionMask = 2;
    CHECK(!server.intersectRay(ray, hit));
    ray.collisionMask = 1;
    CHECK(server.intersectRay(ray, hit) && hit.body == ground);
    ray.collisionMask = 0xFFFFFFFFu;
    ray.exclude = ground;
    CHECK(!server.intersectRay(ray, hit));
}

void testKinematic()
{
    PhysicsServer3D server;
    makeGround(server, 0.3f);
    const PhysicsShapeId shape = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    const PhysicsBodyId pusher =
            makeBody(server, shape, PhysicsBodyMode::Kinematic, at(-3.0f, 0.5f, 0.0f), 0.3f);
    const PhysicsBodyId box =
            makeBody(server, shape, PhysicsBodyMode::Rigid, at(0.0f, 0.5f, 0.0f), 0.3f);
    for (int i = 1; i <= 180; ++i)
    {
        server.bodySetTransform(pusher,
                at(-3.0f + 2.0f * static_cast<float>(i) * 0.0166667f, 0.5f, 0.0f));
        server.step(kStep);
    }
    const float pusherX = server.bodyGetTransform(pusher).col3.x;
    const float boxX = server.bodyGetTransform(box).col3.x;
    printf("  kinematic: pusher x %.3f, box x %.3f\n", pusherX, boxX);
    CHECK(near(pusherX, 3.0f, 0.02f));
    CHECK(boxX > 1.5f);
    CHECK(boxX - pusherX > 0.9f && boxX - pusherX < 1.1f);
}

void testHinge()
{
    PhysicsServer3D server;
    makeGround(server);
    const PhysicsBodyId arm = makeBody(server, boxShape(server, Math::Vec3(1.0f, 0.1f, 0.1f)),
            PhysicsBodyMode::Rigid, at(1.0f, 5.0f, 0.0f));
    const Math::Vec3 anchor(0.0f, 5.0f, 0.0f);
    const PhysicsJointId joint = server.jointCreate(PhysicsJointType::Hinge, PhysicsBodyId(), arm,
            anchor, Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(server.jointValid(joint));
    float worst = 0.0f;
    float lowest = 100.0f;
    for (int i = 0; i < 240; ++i)
    {
        server.step(kStep);
        const Math::Mat4 frame = server.bodyGetTransform(arm);
        const Math::Vec3 pivot = frame.TransformPoint(Math::Vec3(-1.0f, 0.0f, 0.0f));
        worst = fmaxf(worst, (pivot - anchor).Length());
        lowest = fminf(lowest, frame.col3.y);
    }
    printf("  hinge: largest anchor drift %.5f, arm swung down to y %.3f\n", worst, lowest);
    CHECK(worst < 0.03f);
    CHECK(lowest < 4.2f);
    server.jointFree(joint);
    CHECK(!server.jointValid(joint));

    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.3f), PhysicsBodyMode::Rigid,
            at(10.0f, 5.0f, 0.0f));
    const PhysicsJointId pin = server.jointCreate(PhysicsJointType::Pin, PhysicsBodyId(), ball,
            Math::Vec3(10.0f, 7.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));
    worst = 0.0f;
    for (int i = 0; i < 240; ++i)
    {
        server.step(kStep);
        const Math::Vec3 centre = server.bodyGetTransform(ball).col3;
        const float length = (centre - Math::Vec3(10.0f, 7.0f, 0.0f)).Length();
        worst = fmaxf(worst, fabsf(length - 2.0f));
    }
    printf("  pin: largest rope length error %.5f\n", worst);
    CHECK(worst < 0.05f);
    server.jointFree(pin);
}

float runVehicle(PhysicsServer3D& server, PhysicsBodyId& chassisOut, PhysicsVehicleId& vehicleOut)
{
    makeGround(server);
    const PhysicsBodyId chassis = makeBody(server, boxShape(server, Math::Vec3(1.0f, 0.3f, 2.0f)),
            PhysicsBodyMode::Rigid, at(0.0f, 1.2f, 0.0f), 0.8f);
    server.bodySetMass(chassis, 800.0f);
    const PhysicsVehicleId vehicle = server.vehicleCreate(chassis);
    PhysicsWheelParameters wheel;
    wheel.restLength = 0.4f;
    wheel.radius = 0.4f;
    wheel.travel = 0.5f;
    for (int i = 0; i < 4; ++i)
    {
        wheel.connection = Math::Vec3(i % 2 == 0 ? 0.9f : -0.9f, -0.2f, i < 2 ? -1.4f : 1.4f);
        wheel.front = i < 2;
        server.vehicleAddWheel(vehicle, wheel);
    }
    chassisOut = chassis;
    vehicleOut = vehicle;
    run(server, 1.5);
    return yOf(server, chassis);
}

void testVehicle()
{
    PhysicsServer3D server;
    PhysicsBodyId chassis;
    PhysicsVehicleId vehicle;
    const float rest = runVehicle(server, chassis, vehicle);
    printf("  vehicle: rest height %.3f\n", rest);
    CHECK(server.vehicleGetWheelCount(vehicle) == 4);
    CHECK(rest > 0.5f && rest < 1.1f);
    PhysicsWheelState state;
    CHECK(server.vehicleGetWheelState(vehicle, 0, state) && state.inContact);

    for (int i = 0; i < 180; ++i)
    {
        server.vehicleSetEngineForce(vehicle, 2, 900.0f);
        server.vehicleSetEngineForce(vehicle, 3, 900.0f);
        server.step(kStep);
    }
    const Math::Vec3 velocity = server.bodyGetLinearVelocity(chassis);
    printf("  vehicle: after 3 s of engine force velocity (%.3f, %.3f, %.3f), z %.2f\n", velocity.x,
            velocity.y, velocity.z, server.bodyGetTransform(chassis).col3.z);
    CHECK(velocity.z < -4.0f);
    CHECK(fabsf(velocity.x) < 0.3f);

    for (int i = 0; i < 360; ++i)
    {
        for (int w = 0; w < 4; ++w)
        {
            server.vehicleSetEngineForce(vehicle, w, 0.0f);
            server.vehicleSetBrake(vehicle, w, 20.0f);
        }
        server.step(kStep);
    }
    const Math::Vec3 rolling = server.bodyGetLinearVelocity(chassis);
    const float stopped = sqrtf(rolling.x * rolling.x + rolling.z * rolling.z);
    printf("  vehicle: speed after 6 s of braking %.4f\n", stopped);
    CHECK(stopped < 0.05f);

    for (int w = 0; w < 4; ++w) server.vehicleSetBrake(vehicle, w, 0.0f);
    for (int i = 0; i < 180; ++i)
    {
        server.vehicleSetSteering(vehicle, 0, 0.4f);
        server.vehicleSetSteering(vehicle, 1, 0.4f);
        server.vehicleSetEngineForce(vehicle, 2, 900.0f);
        server.vehicleSetEngineForce(vehicle, 3, 900.0f);
        server.step(kStep);
    }
    const Math::Vec3 turned = server.bodyGetLinearVelocity(chassis);
    printf("  vehicle: steering +0.4 rad velocity (%.3f, %.3f, %.3f)\n", turned.x, turned.y,
            turned.z);
    CHECK(turned.x < -0.5f);
}

PhysicsVehicleId makeBike(PhysicsServer3D& server, PhysicsBodyId& chassisOut)
{
    makeGround(server);
    const PhysicsBodyId chassis = makeBody(server, boxShape(server, Math::Vec3(0.2f, 0.3f, 0.9f)),
            PhysicsBodyMode::Rigid, at(0.0f, 1.3f, 0.0f), 0.05f);
    server.bodySetMass(chassis, 200.0f);
    server.bodySetInertia(chassis, Math::Vec3(60.0f, 60.0f, 40.0f));
    server.bodySetCanSleep(chassis, false);
    const PhysicsVehicleId vehicle = server.vehicleCreate(chassis);
    PhysicsWheelParameters wheel;
    wheel.restLength = 0.45f;
    wheel.radius = 0.35f;
    wheel.travel = 0.6f;
    for (int i = 0; i < 2; ++i)
    {
        wheel.connection = Math::Vec3(0.0f, -0.1f, i == 0 ? -0.8f : 0.8f);
        wheel.front = i == 0;
        server.vehicleAddWheel(vehicle, wheel);
    }
    chassisOut = chassis;
    return vehicle;
}

void testMotorcycle()
{
    for (int variant = 0; variant < 2; ++variant)
    {
        PhysicsServer3D server;
        PhysicsBodyId chassis;
        const PhysicsVehicleId bike = makeBike(server, chassis);
        CHECK(!server.vehicleIsMotorcycle(bike));
        CHECK(server.vehicleEnableMotorcycle(bike, true));
        CHECK(server.vehicleIsMotorcycle(bike));
        server.motorcycleSetLeanControllerEnabled(bike, variant == 0);
        server.bodyApplyTorqueImpulse(chassis, Math::Vec3(0.0f, 0.0f, 1.5f));
        run(server, 4.0);
        const Math::Mat4 frame = server.bodyGetTransform(chassis);
        printf("  motorcycle %s: up.y %.3f, height %.2f, lean %.3f\n",
                variant == 0 ? "with lean controller" : "without it", frame.col1.y, frame.col3.y,
                server.motorcycleGetLeanAngle(bike));
        if (variant == 0)
        {
            CHECK(frame.col1.y > 0.95f);
            CHECK(frame.col3.y > 0.2f && frame.col3.y < 1.6f);
        }
        else
            CHECK(frame.col1.y < 0.6f);
    }

    PhysicsServer3D server;
    PhysicsBodyId chassis;
    const PhysicsVehicleId bike = makeBike(server, chassis);
    server.vehicleEnableMotorcycle(bike, true);
    server.motorcycleSetMaxSteerAngle(bike, 0.5f);
    server.motorcycleSetMaxLeanAngle(bike, 0.6f);
    server.motorcycleSetLeanSpring(bike, 6000.0f, 1200.0f);
    run(server, 1.0);
    for (int i = 0; i < 240; ++i)
    {
        server.vehicleSetEngineForce(bike, 1, 700.0f);
        server.motorcycleSetSteerInput(bike, 0.5f);
        server.step(kStep);
    }
    printf("  motorcycle: speed %.1f km/h, up.y %.3f, lean %.3f\n", server.vehicleGetSpeedKmh(bike),
            server.bodyGetTransform(chassis).col1.y, server.motorcycleGetLeanAngle(bike));
    CHECK(fabsf(server.vehicleGetSpeedKmh(bike)) > 5.0f);
    CHECK(server.bodyGetTransform(chassis).col1.y > 0.5f);
    CHECK(fabsf(server.bodyGetTransform(chassis).col3.x) > 0.5f);
    server.vehicleEnableMotorcycle(bike, false);
    CHECK(!server.vehicleIsMotorcycle(bike));
    server.vehicleResetSuspension(bike);
}

struct Humanoid
{
    static constexpr int kBones = 18;
    int parents[kBones];
    Math::Vec3 position[kBones];
    PhysicsRagdollBones map;
};

Humanoid makeHumanoid()
{
    Humanoid h;
    enum
    {
        Hips,
        Spine,
        Neck,
        Head,
        LArm,
        LFore,
        LHand,
        RArm,
        RFore,
        RHand,
        LUp,
        LLeg,
        LFoot,
        LToes,
        RUp,
        RLeg,
        RFoot,
        RToes
    };
    const int parents[Humanoid::kBones] = { -1, Hips, Spine, Neck, Spine, LArm, LFore, Spine, RArm,
        RFore, Hips, LUp, LLeg, LFoot, Hips, RUp, RLeg, RFoot };
    const Math::Vec3 positions[Humanoid::kBones] = { Math::Vec3(0.0f, 1.0f, 0.0f),
        Math::Vec3(0.0f, 1.2f, 0.0f), Math::Vec3(0.0f, 1.5f, 0.0f), Math::Vec3(0.0f, 1.6f, 0.0f),
        Math::Vec3(-0.2f, 1.45f, 0.0f), Math::Vec3(-0.5f, 1.45f, 0.0f),
        Math::Vec3(-0.75f, 1.45f, 0.0f), Math::Vec3(0.2f, 1.45f, 0.0f),
        Math::Vec3(0.5f, 1.45f, 0.0f), Math::Vec3(0.75f, 1.45f, 0.0f),
        Math::Vec3(-0.1f, 0.95f, 0.0f), Math::Vec3(-0.1f, 0.5f, 0.0f),
        Math::Vec3(-0.1f, 0.05f, 0.0f), Math::Vec3(-0.1f, 0.0f, 0.15f),
        Math::Vec3(0.1f, 0.95f, 0.0f), Math::Vec3(0.1f, 0.5f, 0.0f), Math::Vec3(0.1f, 0.05f, 0.0f),
        Math::Vec3(0.1f, 0.0f, 0.15f) };
    for (int i = 0; i < Humanoid::kBones; ++i)
    {
        h.parents[i] = parents[i];
        h.position[i] = positions[i];
    }
    h.map.hips = Hips;
    h.map.spine = Spine;
    h.map.neck = Neck;
    h.map.head = Head;
    h.map.leftArm = LArm;
    h.map.leftForeArm = LFore;
    h.map.leftHand = LHand;
    h.map.rightArm = RArm;
    h.map.rightForeArm = RFore;
    h.map.rightHand = RHand;
    h.map.leftUpLeg = LUp;
    h.map.leftLeg = LLeg;
    h.map.leftFoot = LFoot;
    h.map.leftToes = LToes;
    h.map.rightUpLeg = RUp;
    h.map.rightLeg = RLeg;
    h.map.rightFoot = RFoot;
    h.map.rightToes = RToes;
    return h;
}

Math::Vec3 partPosition(const PhysicsServer3D& server, PhysicsRagdollId ragdoll,
        PhysicsRagdollPart part)
{
    const Math::Mat4 frame = server.ragdollGetPartTransform(ragdoll, part);
    return Math::Vec3(frame.col3.x, frame.col3.y, frame.col3.z);
}

void testRagdoll()
{
    PhysicsServer3D server;
    makeGround(server);
    const Humanoid h = makeHumanoid();

    Math::Mat4 rest[Humanoid::kBones];
    PhysicsBonePose local[Humanoid::kBones];
    for (int i = 0; i < Humanoid::kBones; ++i)
    {
        rest[i] = Math::Mat4::Translation(h.position[i]);
        local[i].position =
                h.parents[i] >= 0 ? h.position[i] - h.position[h.parents[i]] : h.position[i];
    }
    const Math::Mat4 owner = Math::Mat4::Translation(Math::Vec3(0.0f, 2.0f, 0.0f));
    PhysicsRagdollSettings settings;

    PhysicsRagdollBones missing;
    CHECK(!server.ragdollValid(server.ragdollCreate(h.parents, Humanoid::kBones, missing, rest,
            1.0f, rest, owner, settings)));
    PhysicsRagdollBones noToes = h.map;
    noToes.leftToes = -1;
    const PhysicsRagdollId toeless = server.ragdollCreate(h.parents, Humanoid::kBones, noToes, rest,
            1.0f, rest, owner, settings);
    CHECK(server.ragdollValid(toeless));
    server.ragdollFree(toeless);
    PhysicsRagdollBones noNeck = h.map;
    noNeck.neck = -1;
    const PhysicsRagdollId neckless = server.ragdollCreate(h.parents, Humanoid::kBones, noNeck,
            rest, 1.0f, rest, owner, settings);
    CHECK(server.ragdollValid(neckless));
    CHECK(server.ragdollGetPartBone(neckless, PhysicsRagdollPart::Head) == h.map.head);
    server.ragdollFree(neckless);

    const PhysicsRagdollId ragdoll = server.ragdollCreate(h.parents, Humanoid::kBones, h.map, rest,
            1.0f, rest, owner, settings);
    CHECK(server.ragdollValid(ragdoll));
    CHECK(!server.ragdollIsActive(ragdoll));
    CHECK(server.ragdollGetPartBone(ragdoll, PhysicsRagdollPart::Head) == h.map.neck);
    CHECK(server.ragdollGetPartBone(ragdoll, PhysicsRagdollPart::Pelvis) == h.map.hips);
    CHECK(near(server.ragdollGetFacing(ragdoll), -1.0f));

    float massSum = 0.0f;
    for (int part = 0; part < static_cast<int>(PhysicsRagdollPart::Count); ++part)
        massSum += server.ragdollGetPartMass(ragdoll, static_cast<PhysicsRagdollPart>(part));
    printf("  ragdoll: parts add up to %.2f kg\n", massSum);
    CHECK(near(massSum, 13.0f * settings.massScale, 1e-3f));

    const Math::Vec3 pelvisAtRest = partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis);
    CHECK(near(pelvisAtRest.y, 3.0f, 1e-3f));
    const Math::Mat4 movedOwner = Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 0.0f));
    server.ragdollFollow(ragdoll, rest, Humanoid::kBones, movedOwner);
    server.step(kStep);
    const Math::Vec3 pelvisMoved = partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis);
    printf("  ragdoll: follows the owner, pelvis x %.2f -> %.2f\n", pelvisAtRest.x, pelvisMoved.x);
    CHECK(near(pelvisMoved.x, 1.0f, 1e-3f));
    CHECK(near(pelvisMoved.y, 3.0f, 1e-3f));
    CHECK(!server.ragdollIsActive(ragdoll));
    server.ragdollFollow(ragdoll, rest, Humanoid::kBones, owner);
    server.step(kStep);
    CHECK(near(partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis).x, 0.0f, 1e-3f));
    CHECK(server.ragdollGetPartVelocity(ragdoll, PhysicsRagdollPart::Pelvis).Length() > 1.0f);
    server.step(kStep);
    CHECK(near(partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis).x, 0.0f, 1e-3f));
    CHECK(near(server.ragdollGetPartVelocity(ragdoll, PhysicsRagdollPart::Pelvis).Length(), 0.0f,
            1e-4f));

    CHECK(server.ragdollActivate(ragdoll));
    CHECK(server.ragdollIsActive(ragdoll));
    const float headStart = partPosition(server, ragdoll, PhysicsRagdollPart::Head).y;
    server.step(kStep);
    server.ragdollApplyImpulse(ragdoll, PhysicsRagdollPart::Pelvis, Math::Vec3(6.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.1f, 0.0f));
    run(server, 4.0);

    const Math::Vec3 hips = partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis);
    const Math::Vec3 head = partPosition(server, ragdoll, PhysicsRagdollPart::Head);
    printf("  ragdoll: head y %.2f -> %.2f, hips (%.2f %.2f %.2f), head-hips %.2f\n", headStart,
            head.y, hips.x, hips.y, hips.z, (head - hips).Length());
    CHECK(headStart > 3.5f);
    CHECK(head.y < 1.0f);
    CHECK(hips.y < 0.6f && hips.y > 0.0f);
    CHECK(hips.x > 0.3f);
    CHECK((head - hips).Length() < 1.0f);

    run(server, 12.0);
    float fastest = 0.0f;
    float kinetic = 0.0f;
    for (int part = 0; part < static_cast<int>(PhysicsRagdollPart::Count); ++part)
    {
        const float speed =
                server.ragdollGetPartVelocity(ragdoll, static_cast<PhysicsRagdollPart>(part))
                        .Length();
        if (speed > fastest) fastest = speed;
        kinetic += speed * speed;
    }
    printf("  ragdoll: after 16 s the fastest part moves at %.3f m/s\n", fastest);
    CHECK(fastest < 0.2f);
    CHECK(kinetic < 0.2f);

    PhysicsBonePose pose[Humanoid::kBones];
    for (int i = 0; i < Humanoid::kBones; ++i) pose[i] = local[i];
    server.ragdollWritePose(ragdoll, pose, Humanoid::kBones, owner, 1.0f);
    int changed = 0;
    for (int i = 0; i < Humanoid::kBones; ++i)
        if (fabsf(pose[i].rotation.w - 1.0f) > 1e-3f) ++changed;
    printf("  ragdoll: %d bone rotations written\n", changed);
    CHECK(changed >= 8);
    const Math::Vec3 writtenHips = pose[h.map.hips].position;
    const Math::Vec3 hipsNow = partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis);
    printf("  ragdoll: hips bone written at (%.2f %.2f %.2f) in the skeleton frame, body at "
           "(%.2f %.2f %.2f)\n",
            writtenHips.x, writtenHips.y, writtenHips.z, hipsNow.x, hipsNow.y, hipsNow.z);
    CHECK(near(writtenHips.x, hipsNow.x, 1e-3f));
    CHECK(near(writtenHips.y, hipsNow.y - 2.0f, 1e-3f));
    CHECK(near(writtenHips.z, hipsNow.z, 1e-3f));

    server.ragdollDeactivate(ragdoll);
    CHECK(!server.ragdollIsActive(ragdoll));
    server.ragdollFollow(ragdoll, rest, Humanoid::kBones, owner);
    server.step(kStep);
    const Math::Vec3 pelvisBack = partPosition(server, ragdoll, PhysicsRagdollPart::Pelvis);
    printf("  ragdoll: back under the animation, pelvis y %.2f\n", pelvisBack.y);
    CHECK(near(pelvisBack.x, 0.0f, 1e-3f));
    CHECK(near(pelvisBack.y, 3.0f, 1e-3f));
    CHECK(server.ragdollActivate(ragdoll));
    server.ragdollFree(ragdoll);
    CHECK(!server.ragdollValid(ragdoll));
    server.step(kStep);
}

void testShatter()
{
    PhysicsServer3D server;
    const ct::Vector<Math::Vec3> corners = cubeCorners(1.0f, Math::Vec3(0.0f, 0.0f, 0.0f));

    ct::Vector<Math::Vec3> hullVertices;
    ct::Vector<std::uint32_t> hullIndices;
    CHECK(server.convexHullMesh(corners.data(), corners.size(), hullVertices, hullIndices));
    CHECK(hullVertices.size() == 8);
    CHECK(hullIndices.size() == 36);
    CHECK(!server.convexHullMesh(corners.data(), 3, hullVertices, hullIndices));

    ct::Vector<Math::Vec3> sites;
    unsigned seed = 7u;
    for (int i = 0; i < 12; ++i)
    {
        float v[3];
        for (int k = 0; k < 3; ++k)
        {
            seed = seed * 1664525u + 1013904223u;
            v[k] = (static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f * 2.0f - 1.0f) * 0.9f;
        }
        sites.push_back(Math::Vec3(v[0], v[1], v[2]));
    }

    ct::Vector<PhysicsShard> shards;
    CHECK(server.shatter(corners.data(), corners.size(), sites.data(), sites.size(), shards));
    float total = 0.0f;
    bool outward = true;
    bool valid = true;
    for (std::size_t i = 0; i < shards.size(); ++i)
    {
        const PhysicsShard& shard = shards[i];
        total += shard.volume;
        CHECK(fabsf(shard.centroid.x) <= 1.001f && fabsf(shard.centroid.y) <= 1.001f &&
                fabsf(shard.centroid.z) <= 1.001f);
        CHECK(shard.indices.size() % 3 == 0 && shard.indices.size() >= 12);
        for (std::size_t t = 0; t + 2 < shard.indices.size(); t += 3)
        {
            if (shard.indices[t] >= shard.vertices.size() ||
                    shard.indices[t + 1] >= shard.vertices.size() ||
                    shard.indices[t + 2] >= shard.vertices.size())
            {
                valid = false;
                continue;
            }
            const Math::Vec3 a = shard.vertices[shard.indices[t]];
            const Math::Vec3 b = shard.vertices[shard.indices[t + 1]];
            const Math::Vec3 c = shard.vertices[shard.indices[t + 2]];
            if (Math::Vec3::Dot(Math::Vec3::Cross(b - a, c - a), a) < -1e-4f) outward = false;
        }
    }
    printf("  shatter: %zu shards, total volume %.3f of 8\n", shards.size(),
            static_cast<double>(total));
    CHECK(shards.size() >= 6);
    CHECK(fabsf(total - 8.0f) < 0.25f);
    CHECK(valid);
    CHECK(outward);

    PhysicsServer3D::singleton();
    const PhysicsShapeId piece = server.shapeCreate(PhysicsShapeType::ConvexHull);
    server.shapeSetConvexHull(piece, shards[0].vertices.data(), shards[0].vertices.size());
    CHECK(server.shapeGetConvexVertexCount(piece) >= 4);
    CHECK(!server.shatter(corners.data(), corners.size(), nullptr, 0, shards));
}

class SelfRemover : public PhysicsBodyListener
{
public:
    PhysicsServer3D* server = nullptr;
    PhysicsBodyId id;
    int started = 0;
    int moved = 0;

    void physicsContactStarted(const PhysicsContact& contact) override
    {
        (void) contact;
        ++started;
        server->bodyFree(id);
    }

    void physicsBodyMoved(const Math::Mat4& frame) override
    {
        (void) frame;
        ++moved;
    }
};

void clothGrid(int n, float size, float height, ct::Vector<Math::Vec3>& positions,
        ct::Vector<std::uint32_t>& indices)
{
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x)
            positions.push_back(Math::Vec3((static_cast<float>(x) / (n - 1) - 0.5f) * size, height,
                    (static_cast<float>(z) / (n - 1) - 0.5f) * size));
    for (int z = 0; z + 1 < n; ++z)
        for (int x = 0; x + 1 < n; ++x)
        {
            const std::uint32_t a = static_cast<std::uint32_t>(z * n + x);
            const std::uint32_t b = a + 1;
            const std::uint32_t c = a + static_cast<std::uint32_t>(n);
            const std::uint32_t d = c + 1;
            const std::uint32_t quad[6] = { a, c, b, b, c, d };
            for (int k = 0; k < 6; ++k) indices.push_back(quad[k]);
        }
}

void sphereMesh(int rings, int segments, float radius, const Math::Vec3& center,
        ct::Vector<Math::Vec3>& positions, ct::Vector<std::uint32_t>& indices)
{
    positions.push_back(center + Math::Vec3(0.0f, radius, 0.0f));
    for (int r = 1; r < rings; ++r)
    {
        const float phi = 3.14159265f * static_cast<float>(r) / static_cast<float>(rings);
        for (int s = 0; s < segments; ++s)
        {
            const float theta = 6.2831853f * static_cast<float>(s) / static_cast<float>(segments);
            positions.push_back(
                    center + Math::Vec3(radius * sinf(phi) * cosf(theta), radius * cosf(phi),
                                     radius * sinf(phi) * sinf(theta)));
        }
    }
    positions.push_back(center + Math::Vec3(0.0f, -radius, 0.0f));
    const std::uint32_t bottom = static_cast<std::uint32_t>(positions.size() - 1);
    const std::uint32_t segs = static_cast<std::uint32_t>(segments);
    for (std::uint32_t s = 0; s < segs; ++s)
    {
        indices.push_back(0);
        indices.push_back(1 + (s + 1) % segs);
        indices.push_back(1 + s);
    }
    for (int r = 0; r + 2 < rings; ++r)
        for (std::uint32_t s = 0; s < segs; ++s)
        {
            const std::uint32_t a = 1 + static_cast<std::uint32_t>(r) * segs + s;
            const std::uint32_t b = 1 + static_cast<std::uint32_t>(r) * segs + (s + 1) % segs;
            const std::uint32_t c = a + segs;
            const std::uint32_t d = b + segs;
            const std::uint32_t quad[6] = { a, b, c, b, d, c };
            for (int k = 0; k < 6; ++k) indices.push_back(quad[k]);
        }
    const std::uint32_t last = 1 + static_cast<std::uint32_t>(rings - 2) * segs;
    for (std::uint32_t s = 0; s < segs; ++s)
    {
        indices.push_back(bottom);
        indices.push_back(last + s);
        indices.push_back(last + (s + 1) % segs);
    }
}

void softBounds(const PhysicsServer3D& server, PhysicsSoftBodyId body, Math::Vec3& low,
        Math::Vec3& high)
{
    ct::Vector<Math::Vec3> points;
    points.resize(server.softBodyGetParticleCount(body));
    server.softBodyGetPositions(body, points.data(), points.size());
    low = points[0];
    high = points[0];
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        if (points[i].x < low.x) low.x = points[i].x;
        if (points[i].y < low.y) low.y = points[i].y;
        if (points[i].z < low.z) low.z = points[i].z;
        if (points[i].x > high.x) high.x = points[i].x;
        if (points[i].y > high.y) high.y = points[i].y;
        if (points[i].z > high.z) high.z = points[i].z;
    }
}

void testSoftBody()
{
    PhysicsServer3D server;
    makeGround(server);

    ct::Vector<Math::Vec3> positions;
    ct::Vector<std::uint32_t> indices;
    clothGrid(13, 2.0f, 3.0f, positions, indices);

    PhysicsSoftBodyParameters sheet;
    sheet.mass = 1.0f;
    sheet.structuralCompliance = 0.0f;
    sheet.bendCompliance = 0.01f;
    CHECK(!server.softBodyValid(
            server.softBodyCreate(nullptr, 0, indices.data(), indices.size(), sheet)));
    const std::uint32_t outOfRange[3] = { 0, 1, 9999 };
    CHECK(!server.softBodyValid(
            server.softBodyCreate(positions.data(), positions.size(), outOfRange, 3, sheet)));

    const PhysicsSoftBodyId hanging = server.softBodyCreate(positions.data(), positions.size(),
            indices.data(), indices.size(), sheet);
    CHECK(server.softBodyValid(hanging));
    CHECK(server.softBodyGetParticleCount(hanging) == positions.size());
    const std::size_t n = 13;
    const std::size_t corners[4] = { 0, n - 1, n * (n - 1), n * n - 1 };
    for (std::size_t i = 0; i < 4; ++i) server.softBodySetPinned(hanging, corners[i], true);
    CHECK(server.softBodyIsPinned(hanging, 0));
    CHECK(!server.softBodyIsPinned(hanging, 1));
    server.softBodyBuildAttachments(hanging, 1.0f);
    run(server, 3.0);

    Math::Vec3 low, high;
    softBounds(server, hanging, low, high);
    ct::Vector<Math::Vec3> points;
    points.resize(positions.size());
    server.softBodyGetPositions(hanging, points.data(), points.size());
    const float middle = points[(n / 2) * n + n / 2].y;
    printf("  soft cloth: middle y %.3f, corner y %.3f, lowest %.3f, worst stretch %.3f\n", middle,
            points[0].y, low.y, server.softBodyGetWorstStretch(hanging));
    CHECK(near(points[0].y, 3.0f, 1e-3f));
    CHECK(near(points[n * n - 1].y, 3.0f, 1e-3f));
    CHECK(middle < 2.95f);
    CHECK(middle > 1.0f);
    CHECK(server.softBodyGetWorstStretch(hanging) < 1.5f);

    server.softBodySetPinned(hanging, 0, false);
    CHECK(!server.softBodyIsPinned(hanging, 0));
    server.softBodyFree(hanging);
    CHECK(!server.softBodyValid(hanging));
    CHECK(server.softBodyGetParticleCount(hanging) == 0);

    const PhysicsSoftBodyId falling = server.softBodyCreate(positions.data(), positions.size(),
            indices.data(), indices.size(), sheet);
    run(server, 3.0);
    softBounds(server, falling, low, high);
    printf("  soft cloth on ground: y %.3f .. %.3f\n", low.y, high.y);
    CHECK(low.y > -0.02f);
    CHECK(low.y < 0.15f);
    CHECK(high.y < 0.3f);
    server.softBodySetEnabled(falling, false);
    server.softBodyGetPositions(falling, points.data(), 1);
    const Math::Vec3 frozen = points[0];
    run(server, 0.5);
    server.softBodyGetPositions(falling, points.data(), 1);
    CHECK(near(points[0].x, frozen.x) && near(points[0].y, frozen.y) &&
            near(points[0].z, frozen.z));
    server.softBodyFree(falling);

    ct::Vector<Math::Vec3> ballPositions;
    ct::Vector<std::uint32_t> ballIndices;
    sphereMesh(8, 12, 0.5f, Math::Vec3(0.0f, 2.0f, 0.0f), ballPositions, ballIndices);
    PhysicsSoftBodyParameters jelly;
    jelly.mass = 2.0f;
    jelly.structuralCompliance = 0.0001f;
    jelly.bendCompliance = 0.001f;
    jelly.pressure = 40.0f;
    const PhysicsSoftBodyId ball = server.softBodyCreate(ballPositions.data(), ballPositions.size(),
            ballIndices.data(), ballIndices.size(), jelly);
    CHECK(server.softBodyValid(ball));
    CHECK(server.softBodyGetParticleCount(ball) == ballPositions.size());
    run(server, 3.0);
    softBounds(server, ball, low, high);
    printf("  soft jelly: y %.3f .. %.3f, width %.3f, worst stretch %.3f\n", low.y, high.y,
            high.x - low.x, server.softBodyGetWorstStretch(ball));
    CHECK(low.y > -0.02f);
    CHECK(low.y < 0.1f);
    CHECK(high.y - low.y > 0.6f);
    CHECK(high.y - low.y < 1.2f);
    CHECK(high.x - low.x < 1.6f);
    CHECK(server.softBodyGetWorstStretch(ball) < 1.8f);

    server.softBodyAddVelocity(ball, Math::Vec3(0.0f, 4.0f, 0.0f));
    run(server, 0.3);
    softBounds(server, ball, low, high);
    printf("  soft jelly launched: y %.3f .. %.3f\n", low.y, high.y);
    CHECK(low.y > 0.3f);
    server.softBodyFree(ball);

    const PhysicsSoftBodyId duringStep = server.softBodyCreate(positions.data(), positions.size(),
            indices.data(), indices.size(), sheet);
    server.softBodyFree(duringStep);
    server.softBodyFree(duringStep);
    server.step(kStep);
    CHECK(!server.softBodyValid(duringStep));

    const PhysicsSoftBodyId leftover = server.softBodyCreate(positions.data(), positions.size(),
            indices.data(), indices.size(), sheet);
    CHECK(server.softBodyValid(leftover));
}

PhysicsCharacterState walk(PhysicsServer3D& server, PhysicsCharacterId character,
        const Math::Vec3& input, double seconds, bool gravity = true)
{
    PhysicsCharacterState state;
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i)
    {
        Math::Vec3 velocity = server.characterGetVelocity(character);
        velocity.x = input.x;
        velocity.z = input.z;
        if (gravity) velocity.y -= 9.8f * static_cast<float>(kStep);
        else
            velocity.y = input.y;
        server.characterSetVelocity(character, velocity);
        state = server.characterMoveAndSlide(character, static_cast<float>(kStep));
        server.step(kStep);
    }
    return state;
}

void testCharacter()
{
    PhysicsServer3D server;
    makeGround(server);

    PhysicsCharacterParameters parameters;
    parameters.radius = 0.4f;
    parameters.height = 1.2f;
    const PhysicsCharacterId character =
            server.characterCreate(parameters, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(server.characterValid(character));

    PhysicsCharacterState state = walk(server, character, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    printf("  character: landed y %.3f, on floor %d, slope %.1f\n", state.position.y,
            state.onFloor ? 1 : 0, state.slopeAngle);
    CHECK(state.onFloor);
    CHECK(near(state.position.y, 1.0f, 0.06f));
    CHECK(state.floorNormal.y > 0.99f);

    const PhysicsBodyId wall = makeBody(server, boxShape(server, Math::Vec3(0.2f, 2.0f, 5.0f)),
            PhysicsBodyMode::Static, at(4.0f, 2.0f, 0.0f));
    state = walk(server, character, Math::Vec3(4.0f, 0.0f, 0.0f), 1.5);
    printf("  character: against wall x %.3f, on wall %d\n", state.position.x,
            state.onWall ? 1 : 0);
    CHECK(state.onWall);
    CHECK(state.position.x > 3.2f && state.position.x < 3.45f);
    CHECK(fabsf(state.wallNormal.x + 1.0f) < 0.05f);
    CHECK(state.onFloor);
    bool wallHit = false;
    for (std::size_t i = 0; i < server.characterGetHitCount(character); ++i)
        if (server.characterGetHit(character, i).body == wall) wallHit = true;
    CHECK(wallHit);

    server.characterSetPosition(character, Math::Vec3(-6.0f, 1.0f, 0.0f));
    makeBody(server, boxShape(server, Math::Vec3(1.0f, 0.15f, 3.0f)), PhysicsBodyMode::Static,
            at(-3.0f, 0.15f, 0.0f));
    state = walk(server, character, Math::Vec3(2.0f, 0.0f, 0.0f), 3.0);
    printf("  character: after a 0.3 step x %.3f y %.3f\n", state.position.x, state.position.y);
    CHECK(state.position.x > -2.0f);

    server.characterSetPosition(character, Math::Vec3(-6.0f, 1.0f, 8.0f));
    makeBody(server, boxShape(server, Math::Vec3(1.0f, 0.5f, 3.0f)), PhysicsBodyMode::Static,
            at(-3.0f, 0.5f, 8.0f));
    state = walk(server, character, Math::Vec3(2.0f, 0.0f, 0.0f), 3.0);
    printf("  character: against a 1.0 ledge x %.3f\n", state.position.x);
    CHECK(state.position.x < -3.8f);
    CHECK(state.onWall);

    server.characterSetPosition(character, Math::Vec3(8.0f, 1.0f, -8.0f));
    const PhysicsShapeId ballShape = sphereShape(server, 1.0f);
    makeBody(server, ballShape, PhysicsBodyMode::Static, at(11.0f, 1.0f, -8.0f));
    state = walk(server, character, Math::Vec3(3.0f, 0.0f, 0.0f), 2.0);
    printf("  character: against a sphere x %.3f\n", state.position.x);
    CHECK(state.position.x > 9.2f && state.position.x < 9.7f);

    server.characterSetPosition(character, Math::Vec3(8.0f, 1.0f, -14.0f));
    const PhysicsShapeId capsule = server.shapeCreate(PhysicsShapeType::Capsule);
    server.shapeSetCapsule(capsule, 0.5f, 2.0f);
    makeBody(server, capsule, PhysicsBodyMode::Static, at(11.0f, 1.0f, -14.0f));
    state = walk(server, character, Math::Vec3(3.0f, 0.0f, 0.0f), 2.0);
    printf("  character: against a capsule x %.3f\n", state.position.x);
    CHECK(state.position.x > 9.8f && state.position.x < 10.3f);

    server.characterSetPosition(character, Math::Vec3(8.0f, 1.0f, -20.0f));
    makeBody(server, hullShape(server, cubeCorners(1.0f, Math::Vec3(0.0f, 0.0f, 0.0f))),
            PhysicsBodyMode::Static, at(11.0f, 1.0f, -20.0f));
    state = walk(server, character, Math::Vec3(3.0f, 0.0f, 0.0f), 2.0);
    printf("  character: against a hull x %.3f\n", state.position.x);
    CHECK(state.position.x > 9.5f && state.position.x < 9.8f);

    const float tilt = 3.14159265f / 6.0f;
    server.characterSetPosition(character, Math::Vec3(20.0f, 3.0f, 0.0f));
    makeBody(server, boxShape(server, Math::Vec3(6.0f, 0.1f, 4.0f)), PhysicsBodyMode::Static,
            Math::Mat4::Translation(Math::Vec3(20.0f, 1.0f, 0.0f)) * Math::Mat4::RotationZ(tilt));
    state = walk(server, character, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    printf("  character: on a 30 degree ramp, slope %.1f\n", state.slopeAngle);
    CHECK(state.onFloor);
    CHECK(near(state.slopeAngle, 30.0f, 2.0f));

    server.characterSetPosition(character, Math::Vec3(40.0f, 3.0f, 0.0f));
    makeBody(server, boxShape(server, Math::Vec3(6.0f, 0.1f, 4.0f)), PhysicsBodyMode::Static,
            Math::Mat4::Translation(Math::Vec3(40.0f, 1.5f, 0.0f)) *
                    Math::Mat4::RotationZ(3.14159265f / 3.0f));
    state = walk(server, character, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    CHECK(state.onWall || !state.onFloor || state.slopeAngle > 50.0f);

    const PhysicsShapeId crate = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    const PhysicsBodyId box =
            makeBody(server, crate, PhysicsBodyMode::Rigid, at(-6.0f, 0.5f, 20.0f));
    server.characterSetPosition(character, Math::Vec3(-9.0f, 1.0f, 20.0f));
    state = walk(server, character, Math::Vec3(3.0f, 0.0f, 0.0f), 1.0);
    bool crateHit = false;
    for (std::size_t i = 0; i < server.characterGetHitCount(character); ++i)
        if (server.characterGetHit(character, i).body == box) crateHit = true;
    printf("  character: against a crate x %.3f, crate hit %d\n", state.position.x,
            crateHit ? 1 : 0);
    CHECK(crateHit);

    server.characterSetPosition(character, Math::Vec3(0.0f, 1.0f, 0.0f));
    walk(server, character, Math::Vec3(0.0f, 0.0f, 0.0f), 0.2);
    CHECK(server.characterIsNearGround(character, 0.3f));
    server.characterSetPosition(character, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(!server.characterIsNearGround(character, 0.3f));

    PhysicsCharacterParameters ghost = parameters;
    ghost.collisionMask = 0;
    server.characterSetParameters(character, ghost);
    server.characterSetPosition(character, Math::Vec3(0.0f, 3.0f, 0.0f));
    server.characterSetVelocity(character, Math::Vec3(0.0f, 0.0f, 0.0f));
    state = walk(server, character, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    CHECK(state.position.y < -1.0f);

    server.characterSetParameters(character, parameters);
    server.characterSetIgnoredBody(character, wall);
    server.characterSetPosition(character, Math::Vec3(2.0f, 1.0f, 0.0f));
    server.characterSetVelocity(character, Math::Vec3(0.0f, 0.0f, 0.0f));
    state = walk(server, character, Math::Vec3(4.0f, 0.0f, 0.0f), 1.5);
    CHECK(state.position.x > 4.5f);

    server.characterFree(character);
    CHECK(!server.characterValid(character));
    server.characterFree(character);
    server.step(kStep);

    PhysicsServer3D flat;
    const PhysicsShapeId plane = flat.shapeCreate(PhysicsShapeType::WorldBoundary);
    flat.shapeSetWorldBoundary(plane, Math::Vec3(0.0f, 1.0f, 0.0f), 0.0f);
    makeBody(flat, plane, PhysicsBodyMode::Static, Math::Mat4::Identity());
    const PhysicsShapeId ground = flat.shapeCreate(PhysicsShapeType::ConcavePolygon);
    const Math::Vec3 triangle[3] = { Math::Vec3(100.0f, 0.0f, -100.0f),
        Math::Vec3(-100.0f, 0.0f, -100.0f), Math::Vec3(0.0f, 0.0f, 100.0f) };
    const std::uint32_t order[3] = { 0, 1, 2 };
    flat.shapeSetConcaveFaces(ground, triangle, 3, order, 3);
    makeBody(flat, ground, PhysicsBodyMode::Static, at(0.0f, 5.0f, 0.0f));
    const PhysicsCharacterId walker =
            flat.characterCreate(parameters, Math::Vec3(0.0f, 8.0f, 0.0f));
    state = walk(flat, walker, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    printf("  character: on a trimesh over a plane y %.3f\n", state.position.y);
    CHECK(state.onFloor);
    CHECK(near(state.position.y, 6.0f, 0.06f));
    flat.characterSetPosition(walker, Math::Vec3(0.0f, 3.0f, 80.0f));
    flat.characterSetVelocity(walker, Math::Vec3(0.0f, 0.0f, 0.0f));
    state = walk(flat, walker, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    printf("  character: on a plane y %.3f\n", state.position.y);
    CHECK(state.onFloor);
    CHECK(near(state.position.y, 1.0f, 0.06f));
}

PhysicsShapeId cylinderShape(PhysicsServer3D& server, float radius, float height)
{
    const PhysicsShapeId shape = server.shapeCreate(PhysicsShapeType::Cylinder);
    server.shapeSetCylinder(shape, radius, height);
    return shape;
}

void testCylinder()
{
    PhysicsServer3D server;
    makeGround(server);
    const PhysicsShapeId upright = cylinderShape(server, 0.5f, 1.0f);
    CHECK(near(server.shapeGetCylinderRadius(upright), 0.5f));
    CHECK(near(server.shapeGetCylinderHeight(upright), 1.0f));

    const PhysicsBodyId standing =
            makeBody(server, upright, PhysicsBodyMode::Rigid, at(0.0f, 3.0f, 0.0f));
    run(server, 2.5);
    const Math::Mat4 frame = server.bodyGetTransform(standing);
    printf("  cylinder standing: y %.3f, axis y %.3f\n", frame.col3.y, frame.col1.y);
    CHECK(near(frame.col3.y, 0.5f, 0.03f));
    CHECK(frame.col1.y > 0.99f);

    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.3f), PhysicsBodyMode::Rigid,
            at(0.0f, 4.0f, 0.0f));
    run(server, 3.0);
    printf("  sphere on a cylinder: ball y %.3f\n", yOf(server, ball));
    CHECK(near(yOf(server, ball), 1.3f, 0.05f));

    const PhysicsShapeId wheelShape = cylinderShape(server, 0.4f, 0.3f);
    const Math::Mat4 sideways = Math::Mat4::Translation(Math::Vec3(10.0f, 0.4f, 0.0f)) *
                                Math::Mat4::RotationX(3.14159265f * 0.5f);
    const PhysicsBodyId wheel = makeBody(server, wheelShape, PhysicsBodyMode::Rigid, sideways);
    server.bodySetLinearVelocity(wheel, Math::Vec3(3.0f, 0.0f, 0.0f));
    server.bodySetAngularVelocity(wheel, Math::Vec3(0.0f, 0.0f, -7.5f));
    run(server, 2.0);
    const Math::Mat4 rolled = server.bodyGetTransform(wheel);
    printf("  cylinder rolling: x %.2f y %.3f speed %.2f\n", rolled.col3.x, rolled.col3.y,
            server.bodyGetLinearVelocity(wheel).x);
    CHECK(rolled.col3.x > 14.0f);
    CHECK(near(rolled.col3.y, 0.4f, 0.03f));
    CHECK(server.bodyGetLinearVelocity(wheel).x > 2.3f);

    const PhysicsShapeId thin = cylinderShape(server, 1.0f, 0.2f);
    const PhysicsBodyId coin = makeBody(server, thin, PhysicsBodyMode::Rigid,
            Math::Mat4::Translation(Math::Vec3(20.0f, 1.2f, 0.0f)) * Math::Mat4::RotationZ(0.35f));
    run(server, 3.0);
    const Math::Mat4 flat = server.bodyGetTransform(coin);
    printf("  cylinder coin: y %.3f axis y %.3f\n", flat.col3.y, flat.col1.y);
    CHECK(near(flat.col3.y, 0.1f, 0.03f));
    CHECK(fabsf(flat.col1.y) > 0.99f);

    const PhysicsBodyId pillar = makeBody(server, cylinderShape(server, 0.6f, 4.0f),
            PhysicsBodyMode::Static, at(40.0f, 2.0f, 0.0f));
    (void) pillar;
    PhysicsRayParameters ray;
    PhysicsRayResult hit;
    ray.from = Math::Vec3(46.0f, 1.0f, 0.0f);
    ray.to = Math::Vec3(34.0f, 1.0f, 0.0f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 5.4f, 1e-3f) && near(hit.normal.x, 1.0f, 1e-3f));
    ray.from = Math::Vec3(40.2f, 8.0f, 0.1f);
    ray.to = Math::Vec3(40.2f, -2.0f, 0.1f);
    CHECK(server.intersectRay(ray, hit));
    CHECK(near(hit.distance, 4.0f, 1e-3f) && near(hit.normal.y, 1.0f, 1e-3f));
    ray.from = Math::Vec3(40.55f, 8.0f, 0.4f);
    ray.to = Math::Vec3(40.55f, 0.5f, 0.4f);
    CHECK(!server.intersectRay(ray, hit));

    ct::Vector<PhysicsBodyId> found;
    server.overlapSphere(Math::Vec3(41.0f, 1.0f, 0.0f), 0.45f, 0xFFFFFFFFu, found);
    CHECK(found.size() == 1);
    found.clear();
    server.overlapSphere(Math::Vec3(41.2f, 1.0f, 0.0f), 0.45f, 0xFFFFFFFFu, found);
    CHECK(found.empty());

    PhysicsCharacterParameters parameters;
    const PhysicsCharacterId walker =
            server.characterCreate(parameters, Math::Vec3(36.0f, 1.0f, 0.0f));
    PhysicsCharacterState state = walk(server, walker, Math::Vec3(3.0f, 0.0f, 0.0f), 2.0);
    printf("  character against a cylinder: x %.3f\n", state.position.x);
    CHECK(state.position.x > 38.9f && state.position.x < 39.15f);
    server.characterFree(walker);
}

void testFreeDuringStep()
{
    PhysicsServer3D server;
    makeGround(server);
    SelfRemover remover;
    remover.server = &server;
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 2.0f, 0.0f));
    remover.id = ball;
    server.bodySetListener(ball, &remover);
    server.bodySetContactMonitor(ball, true, 4);
    const PhysicsBodyId bystander = makeBody(server, sphereShape(server, 0.5f),
            PhysicsBodyMode::Rigid, at(3.0f, 2.0f, 0.0f));
    run(server, 2.0);
    CHECK(remover.started == 1);
    CHECK(remover.moved > 0);
    CHECK(!server.bodyValid(ball));
    CHECK(server.bodyValid(bystander));
    CHECK(server.getStats().bodies == 2);
    server.step(kStep);
    server.bodyFree(PhysicsBodyId());
}

class Counter : public PhysicsBodyListener
{
public:
    int started = 0;
    int ended = 0;
    void physicsContactStarted(const PhysicsContact& contact) override
    {
        (void) contact;
        ++started;
    }
    void physicsContactEnded(const PhysicsContact& contact) override
    {
        (void) contact;
        ++ended;
    }
};

void testContacts()
{
    PhysicsServer3D server;
    makeGround(server);
    Counter counter;
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 2.0f, 0.0f));
    server.bodySetListener(ball, &counter);
    server.bodySetContactMonitor(ball, true, 2);
    int global = 0;
    server.contactReported = [&global](const PhysicsContactEvent&) { ++global; };
    run(server, 1.5);
    CHECK(counter.started == 1 && counter.ended == 0);
    CHECK(server.bodyGetContactCount(ball) == 1);
    ct::Vector<PhysicsBodyId> touching;
    server.bodyGetCollidingBodies(ball, touching);
    CHECK(touching.size() == 1);
    server.bodyApplyCentralImpulse(ball, Math::Vec3(0.0f, 8.0f, 0.0f));
    run(server, 0.2);
    CHECK(counter.ended == 1);
    CHECK(server.bodyGetContactCount(ball) == 0);
    CHECK(global >= 2);
    server.bodySetInWorld(ball, false);
    CHECK(!server.bodyIsSimulated(ball));
}

void testInterpolation()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    const PhysicsBodyId ball = makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid,
            at(0.0f, 10.0f, 0.0f));
    server.step(kStep);
    const float before = 10.0f;
    const float after = yOf(server, ball);
    const float half = server.bodyGetInterpolatedTransform(ball, 0.5f).col3.y;
    CHECK(near(half, 0.5f * (before + after), 1e-5f));
    CHECK(near(server.bodyGetInterpolatedTransform(ball, 1.0f).col3.y, after, 1e-6f));
    CHECK(near(server.bodyGetInterpolatedTransform(ball, 0.0f).col3.y, before, 1e-6f));
}

void testMeshes()
{
    {
        PhysicsServer3D server;
        const PhysicsShapeId mesh = server.shapeCreate(PhysicsShapeType::ConcavePolygon);
        const Math::Vec3 vertices[8] = { Math::Vec3(-20, 0, -10), Math::Vec3(-20, 0, 10),
            Math::Vec3(0, 0, 10), Math::Vec3(0, 0, -10), Math::Vec3(0, 1, -10),
            Math::Vec3(0, 1, 10), Math::Vec3(20, 1, 10), Math::Vec3(20, 1, -10) };
        const std::uint32_t indices[18] = { 0, 1, 2, 0, 2, 3, 3, 2, 5, 3, 5, 4, 4, 5, 6, 4, 6, 7 };
        server.shapeSetConcaveFaces(mesh, vertices, 8, indices, 18);
        CHECK(server.shapeGetTriangleCount(mesh) == 6);
        makeBody(server, mesh, PhysicsBodyMode::Static, at(0.0f, 0.0f, 0.0f), 0.55f);
        const PhysicsShapeId ballShape = sphereShape(server, 0.5f);
        PhysicsBodyId balls[4];
        const float xs[4] = { -15.0f, -6.0f, 6.0f, 15.0f };
        for (int i = 0; i < 4; ++i)
            balls[i] = makeBody(server, ballShape, PhysicsBodyMode::Rigid,
                    at(xs[i], 4.0f, static_cast<float>(i) - 1.5f));
        run(server, 2.5);
        for (int i = 0; i < 4; ++i)
        {
            const float expected = xs[i] < 0.0f ? 0.5f : 1.5f;
            const float y = yOf(server, balls[i]);
            printf("  trimesh: x %.0f rests at %.3f, surface + radius %.3f\n", xs[i], y, expected);
            CHECK(near(y, expected, 0.03f));
        }
    }
    {
        PhysicsServer3D server;
        const int w = 21;
        const int d = 21;
        ct::Vector<float> heights;
        heights.resize(static_cast<std::size_t>(w * d));
        for (int j = 0; j < d; ++j)
            for (int i = 0; i < w; ++i)
                heights[static_cast<std::size_t>(j * w + i)] = i < 10 ? 0.0f : 2.0f;
        const PhysicsShapeId map = server.shapeCreate(PhysicsShapeType::HeightMap);
        server.shapeSetHeightMap(map, w, d, heights.data());
        CHECK(server.shapeGetTriangleCount(map) == 800);
        makeBody(server, map, PhysicsBodyMode::Static, at(0.0f, 0.0f, 0.0f), 0.55f);
        const PhysicsShapeId ballShape = sphereShape(server, 0.5f);
        PhysicsBodyId balls[4];
        const float xs[4] = { -8.0f, -4.0f, 4.0f, 8.0f };
        for (int i = 0; i < 4; ++i)
            balls[i] = makeBody(server, ballShape, PhysicsBodyMode::Rigid,
                    at(xs[i], 5.0f, static_cast<float>(i) * 2.0f - 3.0f));
        run(server, 2.5);
        for (int i = 0; i < 4; ++i)
        {
            const float expected = xs[i] < 0.0f ? 0.5f : 2.5f;
            const float y = yOf(server, balls[i]);
            printf("  height map: x %.0f rests at %.3f, surface + radius %.3f\n", xs[i], y,
                    expected);
            CHECK(near(y, expected, 0.03f));
        }
    }
}

void populate(PhysicsServer3D& server, ct::Vector<PhysicsBodyId>& bodies)
{
    makeGround(server);
    const PhysicsShapeId box = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    const PhysicsShapeId ball = sphereShape(server, 0.4f);
    for (int i = 0; i < 40; ++i)
    {
        const float x = static_cast<float>(i % 5) * 0.9f;
        const float z = static_cast<float>((i / 5) % 4) * 0.8f;
        const float y = 1.0f + static_cast<float>(i / 20) * 1.5f + static_cast<float>(i % 3) * 0.2f;
        bodies.push_back(makeBody(server, i % 2 ? box : ball, PhysicsBodyMode::Rigid, at(x, y, z)));
    }
}

void testDeterminism()
{
    PhysicsServer3D first;
    ct::Vector<PhysicsBodyId> firstBodies;
    populate(first, firstBodies);
    run(first, 10.0);
    CHECK(first.getStats().steps == 600);

    bool identical = true;
    {
        PhysicsServer3D second;
        ct::Vector<PhysicsBodyId> secondBodies;
        populate(second, secondBodies);
        run(second, 10.0);
        for (std::size_t i = 0; i < firstBodies.size(); ++i)
        {
            const Math::Mat4 a = first.bodyGetTransform(firstBodies[i]);
            const Math::Mat4 b = second.bodyGetTransform(secondBodies[i]);
            if (memcmp(&a, &b, sizeof(Math::Mat4)) != 0) identical = false;
        }
    }
    CHECK(identical);
    printf("  determinism: 40 bodies, 600 steps, identical %s\n", identical ? "yes" : "no");
}

std::size_t liveBytes()
{
    const struct mallinfo2 info = mallinfo2();
    return info.uordblks + info.hblkhd;
}

void ownershipRound(PhysicsServer3D& server)
{
    const PhysicsShapeId shape = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    ct::Vector<PhysicsBodyId> bodies;
    for (int i = 0; i < 1000; ++i)
        bodies.push_back(makeBody(server, shape, PhysicsBodyMode::Rigid,
                at(static_cast<float>(i % 10) * 1.5f, 1.0f + static_cast<float>(i / 100) * 1.2f,
                        static_cast<float>((i / 10) % 10) * 1.5f)));
    server.step(kStep);
    server.step(kStep);
    server.step(kStep);
    for (std::size_t i = 0; i < bodies.size(); ++i) server.bodyFree(bodies[i]);
    server.shapeFree(shape);
    server.step(kStep);
}

void serverLifetime()
{
    PhysicsServer3D server;
    makeGround(server);
    ct::Vector<PhysicsBodyId> bodies;
    populate(server, bodies);
    run(server, 1.0);
}

void testOwnership()
{
    {
        PhysicsServer3D server;
        makeGround(server);
        for (int i = 0; i < 4; ++i) ownershipRound(server);
        const std::size_t before = liveBytes();
        for (int i = 0; i < 16; ++i) ownershipRound(server);
        const std::size_t after = liveBytes();
        const std::size_t drift = after > before ? after - before : before - after;
        printf("  ownership: live bytes before %zu, after %zu, drift %zu\n", before, after, drift);
        CHECK(drift < 1024);
        CHECK(server.getStats().bodies == 1);
    }
    for (int i = 0; i < 30; ++i) serverLifetime();
    const std::size_t baseline = liveBytes();
    for (int i = 0; i < 10; ++i) serverLifetime();
    const std::size_t afterServers = liveBytes();
    printf("  ownership: live bytes around ten server lifetimes after thirty of warm-up %zu -> "
           "%zu\n",
            baseline, afterServers);
    CHECK(afterServers <= baseline + 2048);
    CHECK(PhysicsServer3D::singleton() == nullptr);
}

void testBenchmark()
{
    PhysicsServer3D server;
    makeGround(server);
    const PhysicsShapeId box = boxShape(server, Math::Vec3(0.5f, 0.5f, 0.5f));
    for (int i = 0; i < 1000; ++i)
        makeBody(server, box, PhysicsBodyMode::Rigid,
                at(static_cast<float>(i % 10) * 1.3f, 0.5f + static_cast<float>(i / 100) * 1.1f,
                        static_cast<float>((i / 10) % 10) * 1.3f));
    run(server, 1.0);
    server.resetStats();
    run(server, 3.0);
    const PhysicsStats stats = server.getStats();
    printf("  benchmark: 1000 boxes, 180 steps: average %.3f ms, max %.3f ms, contacts %u, awake "
           "%u\n",
            stats.averageStepMs, stats.maxStepMs, stats.contacts, stats.awakeBodies);
    CHECK(stats.steps == 180);
    CHECK(stats.bodies == 1001);
}

void testDebugDraw()
{
    PhysicsServer3D server;
    makeGround(server);
    makeBody(server, sphereShape(server, 0.5f), PhysicsBodyMode::Rigid, at(0.0f, 2.0f, 0.0f));
    server.step(kStep);
    BatchRenderer batch;
    server.debugDraw(batch);
    CHECK(!batch.empty(BatchRenderer::Space::World));
}

} // namespace

int main()
{
    testFall();
    testConvexHull();
    testConvexHullOnMesh();
    testJointDistance();
    testJointFixed();
    testJointSlider();
    testJointPiston();
    testJointUniversal();
    testJointWheel();
    testJointMouse();
    testJointPinMotor();
    testCollisionExceptions();
    testJointFreeing();
    testBounce();
    testSlope();
    testStack();
    testRaycast();
    testLayers();
    testKinematic();
    testHinge();
    testVehicle();
    testMotorcycle();
    testRagdoll();
    testShatter();
    testSoftBody();
    testCharacter();
    testCylinder();
    testFreeDuringStep();
    testContacts();
    testInterpolation();
    testMeshes();
    testDeterminism();
    testOwnership();
    testBenchmark();
    testDebugDraw();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
