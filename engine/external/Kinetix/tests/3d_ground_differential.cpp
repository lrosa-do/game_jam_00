#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
constexpr f32 kStep = 1.0f / 60.0f;

void run(Scene& scene, Rig& rig, f32 seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5f);
    for (int i = 0; i < steps; ++i)
        step(scene, rig, kStep);
}

// The left half of the world is ice that grips at mu = 0.05 for the car's
// road tyre (0.08 on its table, scaled).
SurfaceInfo splitFriction(u32, const KM::vec3& point, void*)
{
    SurfaceInfo s;
    if (point.x > 0.0f)
    {
        s.type = SurfaceType::Ice;
        s.frictionScale = 0.05f / 0.08f;
    }
    return s;
}

struct SplitResult
{
    f32 speed = 0.0f;
    f32 iceWheelSpeed = 0.0f;
    f32 asphaltWheelSpeed = 0.0f;
    f32 locking = 0.0f;
    f32 iceMu = 0.0f;
};

SplitResult splitLaunch(const DifferentialParameters& diff)
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.vehicle.differential(static_cast<u32>(car.layout.frontDifferential)).setParameters(diff);
    car.vehicle.setSurfaceCallback(splitFriction, nullptr);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 0.4f;
    car.vehicle.setControls(c);
    run(scene, car, 5.0f);
    SplitResult r;
    r.speed = car.vehicle.telemetry().forwardSpeed;
    const f32 radius = car.vehicle.axle(0).wheel.radius;
    r.iceWheelSpeed = car.vehicle.wheel(0).angularVelocity * radius;
    r.asphaltWheelSpeed = car.vehicle.wheel(1).angularVelocity * radius;
    r.locking = car.vehicle.differential(static_cast<u32>(car.layout.frontDifferential)).lockingTorque();
    const Tyre tyre(car.vehicle.axle(0).wheel.tyre);
    r.iceMu = tyre.frictionCoefficient(car.vehicle.wheel(0).load, car.vehicle.wheel(0).surface);
    return r;
}

void splitFrictionLaunch()
{
    DifferentialParameters open;
    open.type = DifferentialType::Open;
    DifferentialParameters locked;
    locked.type = DifferentialType::Locked;
    DifferentialParameters clutch;
    clutch.type = DifferentialType::ClutchLimitedSlip;
    clutch.preloadTorque = 30.0f;
    clutch.torqueSensing = 0.15f;
    DifferentialParameters viscous;
    viscous.type = DifferentialType::ViscousLimitedSlip;
    viscous.viscousCoefficient = 4.0f;
    const SplitResult o = splitLaunch(open);
    const SplitResult l = splitLaunch(locked);
    const SplitResult c = splitLaunch(clutch);
    const SplitResult v = splitLaunch(viscous);
    printf("split friction (ice mu %.3f), 5 s at 40%% throttle in first:\n", double(o.iceMu));
    printf("  open:    %.2f m/s, ice wheel %.1f m/s, asphalt wheel %.2f m/s\n", double(o.speed),
           double(o.iceWheelSpeed), double(o.asphaltWheelSpeed));
    printf("  clutch:  %.2f m/s, ice wheel %.1f m/s, locking %.0f N m\n", double(c.speed), double(c.iceWheelSpeed),
           double(c.locking));
    printf("  viscous: %.2f m/s, ice wheel %.1f m/s, locking %.0f N m\n", double(v.speed), double(v.iceWheelSpeed),
           double(v.locking));
    printf("  locked:  %.2f m/s, wheels %.2f / %.2f m/s\n", double(l.speed), double(l.iceWheelSpeed),
           double(l.asphaltWheelSpeed));
    CHECK_NEAR(o.iceMu, 0.05, 0.01);
    // Open: both wheels get the ice wheel's torque.
    CHECK(o.speed < 1.5f);
    CHECK(o.iceWheelSpeed > 5.0f * fmaxf(o.speed, 0.5f));
    CHECK(o.asphaltWheelSpeed < 1.1f * o.speed + 0.1f);
    CHECK(l.speed > 3.0f * o.speed);
    CHECK_NEAR(l.iceWheelSpeed, l.asphaltWheelSpeed, 1.0e-3);
    CHECK(c.speed > 1.5f * o.speed && c.speed < l.speed);
    CHECK(v.speed > 1.5f * o.speed && v.speed < l.speed);
    CHECK(c.iceWheelSpeed > 1.2f * c.speed);
    CHECK(fabsf(c.locking) > 30.0f);
}

// Wheel speeds in a steady low-speed circle, against the path radii of the
// contact points around the turning centre on the rear axle line.
void corneringWheelSpeeds(DriveLayout drive, bool lockDriven)
{
    Scene scene;
    Rig car;
    PassengerCarOptions options;
    options.drive = drive;
    car.layout = buildPassengerCar(car.vehicle, car.body, options);
    const s32 driven = drive == DriveLayout::FrontWheel ? car.layout.frontDifferential : car.layout.rearDifferential;
    if (lockDriven)
    {
        DifferentialParameters locked;
        locked.type = DifferentialType::Locked;
        car.vehicle.differential(static_cast<u32>(driven)).setParameters(locked);
    }
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.steering = 0.4f;
    for (int i = 0; i < static_cast<int>(16.0f / kStep); ++i)
    {
        const f32 v = car.vehicle.telemetry().forwardSpeed;
        c.throttle = kinetix::Clamp(0.15f + 0.3f * (3.0f - v), 0.0f, 1.0f);
        car.vehicle.setControls(c);
        step(scene, car, kStep);
    }
    ct::Vector<KM::vec3> track;
    for (int i = 0; i < static_cast<int>(8.0f / kStep); ++i)
    {
        const f32 v = car.vehicle.telemetry().forwardSpeed;
        c.throttle = kinetix::Clamp(0.15f + 0.3f * (3.0f - v), 0.0f, 1.0f);
        car.vehicle.setControls(c);
        step(scene, car, kStep);
        track.push_back(car.body.pointToWorld(KM::vec3(0.0f, 0.0f, car.vehicle.referenceAxlePosition())));
    }
    f32 cx, cz, radius;
    fitCircle(track, cx, cz, radius);
    const f32 L = car.vehicle.wheelbase();
    const f32 h = car.vehicle.axle(0).halfTrack;
    // Turning right: the left wheels are outside.
    const f32 frontRatio = sqrtf((radius + h) * (radius + h) + L * L) / sqrtf((radius - h) * (radius - h) + L * L);
    const f32 rearRatio = (radius + h) / (radius - h);
    const f32 front = car.vehicle.wheel(0).angularVelocity / car.vehicle.wheel(1).angularVelocity;
    const f32 rear = car.vehicle.wheel(2).angularVelocity / car.vehicle.wheel(3).angularVelocity;
    const char* name = drive == DriveLayout::FrontWheel ? "front-drive" : "rear-drive";
    printf("%s, %s differential, circle of %.2f m at %.2f m/s: outer/inner wheel speed front %.4f (paths %.4f), "
           "rear %.4f (paths %.4f)\n",
           name, lockDriven ? "locked" : "open", double(radius), double(car.vehicle.telemetry().forwardSpeed),
           double(front), double(frontRatio), double(rear), double(rearRatio));
    if (!lockDriven)
    {
        CHECK_NEAR(front, frontRatio, 0.01 * frontRatio);
        CHECK_NEAR(rear, rearRatio, 0.01 * rearRatio);
    }
    else
    {
        const f32 lockedRatio = drive == DriveLayout::FrontWheel ? front : rear;
        const f32 freeRatio = drive == DriveLayout::FrontWheel ? rear : front;
        const f32 freePaths = drive == DriveLayout::FrontWheel ? rearRatio : frontRatio;
        CHECK_NEAR(lockedRatio, 1.0, 1.0e-3);
        CHECK_NEAR(freeRatio, freePaths, 0.02 * freePaths);
    }
    CHECK(radius > 4.0f && radius < 14.0f);
}

void allWheelDriveSplit()
{
    Scene scene;
    Rig car;
    PassengerCarOptions options;
    options.drive = DriveLayout::AllWheel;
    car.layout = buildPassengerCar(car.vehicle, car.body, options);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.launch(10.0f);
    car.vehicle.selectGear(2);
    VehicleControls c;
    c.throttle = 0.6f;
    car.vehicle.setControls(c);
    run(scene, car, 2.0f);
    f32 front = 0.0f;
    f32 rear = 0.0f;
    for (int i = 0; i < 30; ++i)
    {
        step(scene, car, kStep);
        front += car.vehicle.differential(static_cast<u32>(car.layout.frontDifferential)).inputTorque();
        rear += car.vehicle.differential(static_cast<u32>(car.layout.rearDifferential)).inputTorque();
    }
    const f32 share = front / (front + rear);
    printf("all-wheel drive at 60%% in second: front share of the drive torque %.3f (centre split 0.40), "
           "wheel drive torques %.0f %.0f %.0f %.0f N m\n",
           double(share), double(car.vehicle.wheel(0).driveTorque), double(car.vehicle.wheel(1).driveTorque),
           double(car.vehicle.wheel(2).driveTorque), double(car.vehicle.wheel(3).driveTorque));
    CHECK_NEAR(share, 0.4, 0.01);
    CHECK(fabsf(front + rear) > 100.0f);
    CHECK_NEAR(car.vehicle.wheel(2).driveTorque, car.vehicle.wheel(3).driveTorque,
               0.02 * fabsf(car.vehicle.wheel(2).driveTorque) + 1.0);
    car.vehicle.setDifferentialLock(static_cast<u32>(car.layout.centreDifferential), true);
    run(scene, car, 1.0f);
    const f32 frontSpeed = car.vehicle.differential(static_cast<u32>(car.layout.frontDifferential)).inputAngularVelocity();
    const f32 rearSpeed = car.vehicle.differential(static_cast<u32>(car.layout.rearDifferential)).inputAngularVelocity();
    CHECK_NEAR(frontSpeed, rearSpeed, 1.0e-3 * fabsf(rearSpeed) + 1.0e-4);
}

void invalidTopologyIsUnpowered()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    DifferentialParameters p;
    const u32 bad = car.vehicle.addDifferential(p, DriveOutput::wheel(17), DriveOutput::wheel(1));
    car.vehicle.setDriveRoot(static_cast<s32>(bad));
    car.place(scene, KM::vec3(0.0f));
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 1.0f;
    car.vehicle.setControls(c);
    run(scene, car, 1.0f);
    CHECK(fabsf(car.vehicle.telemetry().forwardSpeed) < 0.01f);
    CHECK(car.vehicle.telemetry().engineRpm == 0.0f);
    CHECK(isfinite(car.body.position().y));
}
}

int main()
{
    splitFrictionLaunch();
    corneringWheelSpeeds(DriveLayout::FrontWheel, false);
    corneringWheelSpeeds(DriveLayout::RearWheel, false);
    corneringWheelSpeeds(DriveLayout::RearWheel, true);
    allWheelDriveSplit();
    invalidTopologyIsUnpowered();
    return TEST_RESULT();
}
