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

void ackermannGeometry()
{
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    const f32 L = car.vehicle.wheelbase();
    const f32 h = car.vehicle.axle(0).halfTrack;
    CHECK_NEAR(L, car.layout.wheelbase, 1.0e-5);
    f32 worst = 0.0f;
    const f32 angles[4] = {0.05f, 0.2f, 0.4f, 0.6f};
    for (f32 delta : angles)
    {
        const f32 R = L / tanf(delta);
        const f32 inner = atanf(L / (R - h));
        const f32 outer = atanf(L / (R + h));
        const f32 left = car.vehicle.wheelSteerAngle(0, delta);
        const f32 right = car.vehicle.wheelSteerAngle(1, delta);
        const f32 leftL = car.vehicle.wheelSteerAngle(0, -delta);
        const f32 rightL = car.vehicle.wheelSteerAngle(1, -delta);
        printf("Ackermann at %.2f rad: inner %.4f (tan geometry %.4f), outer %.4f (%.4f)\n", double(delta),
               double(right), double(inner), double(left), double(outer));
        worst = fmaxf(worst, fabsf(right - inner));
        worst = fmaxf(worst, fabsf(left - outer));
        worst = fmaxf(worst, fabsf(-leftL - inner));
        worst = fmaxf(worst, fabsf(-rightL - outer));
        CHECK(right > delta && left < delta);
        CHECK(car.vehicle.wheelSteerAngle(2, delta) == 0.0f);
    }
    CHECK(worst < 1.0e-5f);
    SteeringParameters parallel = car.vehicle.steering();
    parallel.ackermann = 0.0f;
    car.vehicle.setSteering(parallel);
    CHECK_NEAR(car.vehicle.wheelSteerAngle(0, 0.3f), 0.3f, 1.0e-5);
    CHECK_NEAR(car.vehicle.wheelSteerAngle(1, 0.3f), 0.3f, 1.0e-5);
}

void steeringRateLimit()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 0.5f);
    VehicleControls c;
    c.steering = 1.0f;
    car.vehicle.setControls(c);
    const SteeringParameters& s = car.vehicle.steering();
    run(scene, car, 0.2f);
    const f32 after = car.vehicle.telemetry().steeringAngle;
    run(scene, car, 0.5f);
    printf("steering step: %.3f rad after 0.2 s (rate %.2f rad/s), %.3f after 0.7 s (max %.2f)\n", double(after),
           double(s.rate), double(car.vehicle.telemetry().steeringAngle), double(s.maxAngle));
    CHECK_NEAR(after, s.rate * 0.2f, s.rate * kStep + 1.0e-4);
    CHECK_NEAR(car.vehicle.telemetry().steeringAngle, s.maxAngle, 1.0e-5);
    CHECK_NEAR(car.vehicle.wheel(1).steerAngle, car.vehicle.wheelSteerAngle(1, s.maxAngle), 1.0e-5);
}

// Drives a steady circle at `speed` with the reference angle `steer` (of
// full lock); returns the radius the reference point (middle of the
// non-steered axles) traces, and the steady yaw rate and roll.
struct Circle
{
    f32 radius = 0.0f;
    f32 yawRate = 0.0f;
    f32 roll = 0.0f;
    f32 speed = 0.0f;
    f32 delta = 0.0f;
};

Circle steadyCircle(Rig& rig, Scene& scene, f32 steer, f32 speed, f32 settle = 14.0f, f32 measure = 8.0f)
{
    rig.vehicle.selectGear(rig.vehicle.gearbox().mode() == GearboxMode::Automatic ? 1 : 2);
    if (speed < 4.0f)
        rig.vehicle.selectGear(1);
    rig.launch(speed);
    VehicleControls c;
    c.steering = steer;
    auto control = [&]()
    {
        const f32 v = rig.vehicle.telemetry().forwardSpeed;
        c.throttle = kinetix::Clamp(0.2f + 0.5f * (speed - v), 0.0f, 1.0f);
        rig.vehicle.setControls(c);
    };
    for (int i = 0; i < static_cast<int>(settle / kStep); ++i)
    {
        control();
        step(scene, rig, kStep);
    }
    ct::Vector<KM::vec3> track;
    f32 yaw = 0.0f;
    f32 roll = 0.0f;
    f32 v = 0.0f;
    const int n = static_cast<int>(measure / kStep);
    for (int i = 0; i < n; ++i)
    {
        control();
        step(scene, rig, kStep);
        track.push_back(rig.body.pointToWorld(KM::vec3(0.0f, 0.0f, rig.vehicle.referenceAxlePosition())));
        yaw += rig.vehicle.telemetry().yawRate;
        roll += rig.vehicle.telemetry().roll;
        v += rig.vehicle.telemetry().forwardSpeed;
    }
    Circle out;
    f32 cx, cz;
    fitCircle(track, cx, cz, out.radius);
    out.yawRate = yaw / n;
    out.roll = roll / n;
    out.speed = v / n;
    out.delta = rig.vehicle.telemetry().steeringAngle;
    return out;
}

void lowSpeedTurningRadius()
{
    const f32 steers[3] = {0.2f, 0.5f, -0.5f};
    for (f32 steer : steers)
    {
        Scene scene;
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.place(scene, KM::vec3(0.0f));
        run(scene, car, 0.5f);
        const Circle c = steadyCircle(car, scene, steer, 2.5f);
        const f32 theory = car.vehicle.wheelbase() / tanf(fabsf(c.delta));
        printf("car at %.2f m/s, steering %+.1f (%.3f rad): radius %.2f m, wheelbase/tan(delta) %.2f m (%+.2f%%)\n",
               double(c.speed), double(steer), double(c.delta), double(c.radius), double(theory),
               double(100.0f * (c.radius - theory) / theory));
        CHECK_NEAR(c.radius, theory, 0.05 * theory);
        // Turning the way it is steered: yaw rate positive to the right.
        CHECK(c.yawRate * steer > 0.0f);
    }
}

// The bicycle-model understeer gradient from two speeds on the same
// steering: K = (L/R_slow - L/R_fast) / a_y,fast, rad per m/s^2.
f32 understeerGradient(f32 frontShift, bool antiRoll, f32* theory)
{
    f32 inverseRadius[2];
    f32 lateral = 0.0f;
    const f32 speeds[2] = {2.5f, 13.0f};
    Rig* last = nullptr;
    Scene* lastScene = nullptr;
    f32 L = 0.0f;
    for (u32 k = 0; k < 2; ++k)
    {
        Scene* scene = new Scene();
        Rig* car = new Rig();
        car->layout = buildPassengerCar(car->vehicle, car->body);
        // Move the centre of mass by sliding both axles forward (positive
        // shift makes the car rear-heavy), and re-level it for the new
        // axle loads.
        const f32 frontZ = car->vehicle.axle(0).position + frontShift;
        const f32 rearZ = car->vehicle.axle(1).position + frontShift;
        const f32 weight = car->layout.mass * kG;
        const f32 frontLoad = weight * (-rearZ) / (frontZ - rearZ);
        const f32 loads[2] = {frontLoad, weight - frontLoad};
        for (u32 a = 0; a < 2; ++a)
        {
            AxleParameters& axle = car->vehicle.axle(a);
            axle.position += frontShift;
            axle.wheel.suspension =
                SuspensionParameters::forLoad(0.5f * loads[a], 1.4f, 0.35f, 0.2f, 0.6f);
            axle.height = axle.wheel.radius + axle.wheel.suspension.travel -
                          axle.wheel.suspension.staticCompression(0.5f * loads[a]) - car->layout.restHeight;
            if (!antiRoll)
                axle.antiRollStiffness = 0.0f;
        }
        car->place(*scene, KM::vec3(0.0f));
        run(*scene, *car, 1.0f);
        const Circle c = steadyCircle(*car, *scene, 0.1f, speeds[k], 12.0f, 6.0f);
        L = car->vehicle.wheelbase();
        inverseRadius[k] = c.yawRate / c.speed;
        if (k == 1)
            lateral = c.yawRate * c.speed;
        if (theory)
        {
            // W/C per axle with the tyre model's cornering stiffness at the
            // static wheel loads.
            const Tyre tyre(car->vehicle.axle(0).wheel.tyre);
            const f32 cf = 2.0f * tyre.corneringStiffness(0.5f * loads[0], SurfaceInfo());
            const f32 cr = 2.0f * tyre.corneringStiffness(0.5f * loads[1], SurfaceInfo());
            *theory = loads[0] / cf - loads[1] / cr;
            *theory /= kG; // per m/s^2
        }
        delete last;
        delete lastScene;
        last = car;
        lastScene = scene;
    }
    delete last;
    delete lastScene;
    return L * (inverseRadius[0] - inverseRadius[1]) / lateral;
}

void understeer()
{
    f32 theoryFront = 0.0f;
    f32 theoryRear = 0.0f;
    const f32 frontHeavy = understeerGradient(0.0f, false, &theoryFront);
    const f32 rearHeavy = understeerGradient(0.52f, false, &theoryRear);
    const f32 standard = understeerGradient(0.0f, true, nullptr);
    const f32 perG = kG / kDeg;
    printf("understeer gradient: 60/40 car %.2f deg/g (tyre-only theory %.2f), 40/60 car %.2f deg/g (%.2f), "
           "60/40 with its anti-roll bars %.2f deg/g\n",
           double(frontHeavy * perG), double(theoryFront * perG), double(rearHeavy * perG),
           double(theoryRear * perG), double(standard * perG));
    // Moving the weight back takes understeer away by about what the axles' W/C terms say; the measured gradient also carries
    // aligning moments, roll-stiffness split and Ackermann under load transfer, all adding understeer, so the 40/60 car ends
    // close to neutral rather than at the tyre-only theory's oversteer.
    const f32 change = rearHeavy - frontHeavy;
    const f32 theoryChange = theoryRear - theoryFront;
    printf("  moving the weight back: %.2f deg/g measured, %.2f deg/g from W/C\n", double(change * perG),
           double(theoryChange * perG));
    CHECK(frontHeavy > 0.0f);
    CHECK(frontHeavy * theoryFront > 0.0f);
    CHECK(theoryChange < 0.0f);
    CHECK(change < 0.0f);
    CHECK(fabsf(change - theoryChange) < 0.5f * fabsf(theoryChange));
    CHECK(standard > 0.0f && isfinite(standard));
}

void antiRollBars()
{
    f32 rolls[3];
    const f32 scales[3] = {0.0f, 1.0f, 4.0f};
    for (u32 k = 0; k < 3; ++k)
    {
        Scene scene;
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        for (u32 a = 0; a < 2; ++a)
            car.vehicle.axle(a).antiRollStiffness *= scales[k];
        car.place(scene, KM::vec3(0.0f));
        run(scene, car, 1.0f);
        const Circle c = steadyCircle(car, scene, 0.1f, 12.0f, 10.0f, 4.0f);
        rolls[k] = c.roll;
        printf("anti-roll bars x%.0f: roll %.3f deg at %.2f m/s^2\n", double(scales[k]), double(c.roll / kDeg),
               double(c.yawRate * c.speed));
    }
    // Turning right rolls the body out to the left (roll negative).
    CHECK(rolls[0] < 0.0f);
    CHECK(fabsf(rolls[1]) < fabsf(rolls[0]));
    CHECK(fabsf(rolls[2]) < fabsf(rolls[1]));
    CHECK(fabsf(rolls[2]) < 0.6f * fabsf(rolls[0]));
}

void eightByFour()
{
    Scene scene;
    Rig truck;
    truck.layout = buildTruck8x4(truck.vehicle, truck.body);
    const f32 reference = truck.vehicle.referenceAxlePosition();
    const f32 L = truck.vehicle.wheelbase();
    const f32 d1 = truck.vehicle.axle(0).position - reference;
    const f32 d2 = truck.vehicle.axle(1).position - reference;
    const f32 h = truck.vehicle.axle(0).halfTrack;
    const f32 delta = 0.3f;
    const f32 R = L / tanf(delta);
    const f32 first = truck.vehicle.wheelSteerAngle(1, delta);
    const f32 second = truck.vehicle.wheelSteerAngle(3, delta);
    printf("8x4: steer axles %.2f and %.2f m ahead of the tandem; inner wheels %.4f and %.4f rad "
           "(tan geometry %.4f, %.4f)\n",
           double(d1), double(d2), double(first), double(second), double(atanf(d1 / (R - h))),
           double(atanf(d2 / (R - h))));
    CHECK_NEAR(d1, L, 1.0e-5);
    CHECK_NEAR(first, atanf(d1 / (R - h)), 1.0e-5);
    CHECK_NEAR(second, atanf(d2 / (R - h)), 1.0e-5);
    CHECK(second < first);
    CHECK(truck.vehicle.wheelSteerAngle(4, delta) == 0.0f);
    truck.place(scene, KM::vec3(0.0f));
    run(scene, truck, 1.0f);
    const Circle c = steadyCircle(truck, scene, 0.5f, 2.0f, 16.0f, 10.0f);
    const f32 theory = L / tanf(c.delta);
    printf("8x4 at %.2f m/s: radius %.2f m about the tandem's middle, wheelbase/tan(delta) %.2f m (%+.1f%%)\n",
           double(c.speed), double(c.radius), double(theory), double(100.0f * (c.radius - theory) / theory));
    CHECK_NEAR(c.radius, theory, 0.08 * theory);
}
}

int main()
{
    ackermannGeometry();
    steeringRateLimit();
    lowSpeedTurningRadius();
    understeer();
    antiRollBars();
    eightByFour();
    return TEST_RESULT();
}
