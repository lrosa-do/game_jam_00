#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
constexpr f32 kStep = 1.0f / 60.0f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;

void run(Scene& scene, Rig& rig, f32 seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5f);
    for (int i = 0; i < steps; ++i)
        step(scene, rig, kStep);
}

void engineControl()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 3.0f);
    const EngineParameters& e = car.vehicle.engine().parameters();
    const f32 idle = car.vehicle.telemetry().engineRpm;
    printf("idle in neutral: %.0f rpm (set %.0f)\n", double(idle), double(e.idleRpm));
    CHECK_NEAR(idle, e.idleRpm, 0.05 * e.idleRpm);

    VehicleControls c;
    c.throttle = 1.0f;
    car.vehicle.setControls(c);
    f32 peak = 0.0f;
    f32 sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < static_cast<int>(4.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        peak = fmaxf(peak, car.vehicle.telemetry().engineRpm);
        if (i > static_cast<int>(2.0f / kStep))
        {
            sum += car.vehicle.telemetry().engineRpm;
            ++samples;
        }
    }
    printf("free revving: peak %.0f rpm, mean %.0f (limiter %.0f)\n", double(peak), double(sum / samples),
           double(e.limiterRpm));
    CHECK(peak < e.limiterRpm + 250.0f);
    CHECK_NEAR(sum / samples, e.limiterRpm, 0.03 * e.limiterRpm);

    c.throttle = 0.0f;
    car.vehicle.setControls(c);
    f32 backToIdle = -1.0f;
    for (int i = 0; i < static_cast<int>(4.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        if (backToIdle < 0.0f && car.vehicle.telemetry().engineRpm < 1.05f * e.idleRpm)
            backToIdle = (i + 1) * kStep;
    }
    printf("throttle closed at the limiter: within 5%% of idle after %.2f s, %.0f rpm after 4 s\n",
           double(backToIdle), double(car.vehicle.telemetry().engineRpm));
    CHECK(backToIdle > 0.0f && backToIdle < 3.0f);
    CHECK_NEAR(car.vehicle.telemetry().engineRpm, e.idleRpm, 0.05 * e.idleRpm);
    car.vehicle.engine().stop();
    run(scene, car, 3.0f);
    CHECK(!car.vehicle.telemetry().engineRunning);
    CHECK(car.vehicle.telemetry().engineRpm < 1.0f);
    c.starter = true;
    car.vehicle.setControls(c);
    run(scene, car, 1.0f);
    c.starter = false;
    car.vehicle.setControls(c);
    run(scene, car, 2.0f);
    printf("after the starter: running %d at %.0f rpm\n", car.vehicle.telemetry().engineRunning,
           double(car.vehicle.telemetry().engineRpm));
    CHECK(car.vehicle.telemetry().engineRunning);
    CHECK_NEAR(car.vehicle.telemetry().engineRpm, e.idleRpm, 0.05 * e.idleRpm);
}

// Every gear held at full throttle until the limiter: the road speed it
// settles at is the limiter speed through the gear, final drive and tyre.
void topSpeedPerGear(const char* name, Rig& rig, Scene& scene, u32 gears, f32 timeout)
{
    const EngineParameters& e = rig.vehicle.engine().parameters();
    const f32 radius = rig.vehicle.axle(0).wheel.radius;
    const f32 rearRadius = rig.vehicle.axle(rig.vehicle.axleCount() - 1).wheel.radius;
    // The driven wheels' radius: the tractor's rear tyres set its speed.
    const f32 r = fmaxf(radius, rearRadius);
    VehicleControls c;
    c.throttle = 1.0f;
    rig.vehicle.setControls(c);
    rig.vehicle.selectGear(1);
    f32 worst = 0.0f;
    for (u32 gear = 1; gear <= gears; ++gear)
    {
        if (gear > 1)
            rig.vehicle.shiftUp();
        run(scene, rig, 1.0f);
        for (int i = 0; i < static_cast<int>(timeout / kStep); ++i)
        {
            step(scene, rig, kStep);
            if (rig.vehicle.telemetry().engineRpm > e.limiterRpm - e.limiterBand)
                break;
        }
        for (int k = 0; k < 20; ++k)
        {
            const f32 before = rig.vehicle.telemetry().forwardSpeed;
            run(scene, rig, 0.5f);
            if (fabsf(rig.vehicle.telemetry().forwardSpeed - before) < 1.0e-3f * fabsf(before) + 1.0e-4f)
                break;
        }
        f32 sum = 0.0f;
        int samples = 0;
        for (int i = 0; i < static_cast<int>(1.0f / kStep); ++i)
        {
            step(scene, rig, kStep);
            sum += rig.vehicle.telemetry().forwardSpeed;
            ++samples;
        }
        const f32 measured = sum / samples;
        const f32 ratio = rig.vehicle.gearbox().ratioOf(static_cast<s32>(gear));
        const f32 theory = e.limiterRpm * kRpmToRadians * r / ratio;
        const f32 error = (measured - theory) / theory;
        worst = fmaxf(worst, fabsf(error));
        printf("%s gear %u: %.2f m/s at the limiter, theory %.2f (%+.2f%%), %.0f rpm\n", name, gear, double(measured),
               double(theory), double(100.0f * error), double(rig.vehicle.telemetry().engineRpm));
        CHECK(rig.vehicle.telemetry().gear == static_cast<s32>(gear));
        CHECK(fabsf(error) < 0.02f);
    }
    printf("%s: worst per-gear top speed error %.2f%%\n", name, double(100.0f * worst));
}

void carTopSpeeds()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    topSpeedPerGear("car", car, scene, 3, 20.0f);
}

void tractorTopSpeeds()
{
    Scene scene;
    Rig tractor;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    tractor.place(scene, KM::vec3(0.0f));
    run(scene, tractor, 1.0f);
    topSpeedPerGear("tractor", tractor, scene, 10, 10.0f);
}

void clutchLaunchAndStall()
{
    {
        Scene scene;
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.place(scene, KM::vec3(0.0f));
        run(scene, car, 1.0f);
        car.vehicle.selectGear(1);
        VehicleControls c;
        c.throttle = 0.35f;
        car.vehicle.setControls(c);
        f32 lowest = 1.0e9f;
        f32 lockTime = -1.0f;
        int relocks = 0;
        for (int i = 0; i < static_cast<int>(5.0f / kStep); ++i)
        {
            step(scene, car, kStep);
            const VehicleTelemetry& t = car.vehicle.telemetry();
            if (i > 20)
                lowest = fminf(lowest, t.engineRpm);
            if (lockTime < 0.0f && t.clutchLocked)
                lockTime = (i + 1) * kStep;
            if (lockTime > 0.0f && !t.clutchLocked)
                ++relocks;
        }
        const VehicleTelemetry& t = car.vehicle.telemetry();
        printf("launch at 35%% throttle: lowest %.0f rpm, clutch locked after %.2f s (slipped again on %d steps), "
               "%.2f m/s after 5 s, engagement %.2f\n",
               double(lowest), double(lockTime), relocks, double(t.forwardSpeed), double(t.clutchEngagement));
        CHECK(t.engineRunning);
        CHECK(relocks == 0);
        CHECK(lowest > car.vehicle.engine().parameters().idleRpm);
        CHECK(lockTime > 0.0f && lockTime < 5.0f);
        // 35% of a linear throttle map nets ~28 N m at 1200 rpm: about
        // 0.8 m/s^2 in first.
        CHECK(t.forwardSpeed > 2.5f);
    }
    {
        Scene scene;
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.vehicle.setAutoClutch(false);
        car.place(scene, KM::vec3(0.0f));
        VehicleControls c;
        c.clutch = 1.0f;
        car.vehicle.setControls(c);
        car.vehicle.selectGear(1);
        run(scene, car, 1.0f);
        CHECK(car.vehicle.telemetry().gear == 1);
        CHECK(car.vehicle.telemetry().engineRunning);
        const KM::vec3 start = car.body.position();
        c.clutch = 0.0f;
        car.vehicle.setControls(c);
        f32 stallTime = -1.0f;
        for (int i = 0; i < static_cast<int>(2.0f / kStep); ++i)
        {
            step(scene, car, kStep);
            if (stallTime < 0.0f && !car.vehicle.telemetry().engineRunning)
                stallTime = (i + 1) * kStep;
        }
        const f32 moved = horizontalDistance(car.body.position(), start);
        printf("clutch dumped at idle: stalled after %.2f s, the car moved %.2f m\n", double(stallTime),
               double(moved));
        CHECK(stallTime > 0.0f && stallTime < 0.5f);
        CHECK(!car.vehicle.telemetry().engineRunning);
        CHECK(moved < 0.5f);
        c.clutch = 1.0f;
        c.starter = true;
        car.vehicle.setControls(c);
        run(scene, car, 1.0f);
        CHECK(car.vehicle.telemetry().engineRunning);
    }
}

void noChatterAtLock()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.launch(15.0f);
    car.vehicle.selectGear(3);
    VehicleControls c;
    c.throttle = 0.4f;
    car.vehicle.setControls(c);
    run(scene, car, 3.0f);
    f32 sum = 0.0f;
    f32 sumSq = 0.0f;
    f32 previous = car.vehicle.telemetry().clutchTorque;
    f32 worstJump = 0.0f;
    int locked = 0;
    const int samples = static_cast<int>(3.0f / kStep);
    for (int i = 0; i < samples; ++i)
    {
        step(scene, car, kStep);
        const f32 torque = car.vehicle.telemetry().clutchTorque;
        sum += torque;
        sumSq += torque * torque;
        worstJump = fmaxf(worstJump, fabsf(torque - previous));
        previous = torque;
        if (car.vehicle.telemetry().clutchLocked)
            ++locked;
    }
    const f32 mean = sum / samples;
    const f32 deviation = sqrtf(fmaxf(sumSq / samples - mean * mean, 0.0f));
    printf("locked clutch in third at 40%%: %.1f N m mean, %.2f N m deviation, worst step change %.2f N m, "
           "locked %d / %d steps, slip %.4f rad/s\n",
           double(mean), double(deviation), double(worstJump), locked, samples,
           double(car.vehicle.telemetry().clutchSlip));
    CHECK(locked == samples);
    CHECK(mean > 20.0f);
    CHECK(deviation < 0.02f * mean);
    CHECK(worstJump < 0.01f * mean);
    CHECK(fabsf(car.vehicle.telemetry().clutchSlip) < 1.0e-3f);

    // Efficiency: the driven wheels get efficiency x ratio x the clutch
    // torque, less the box's small drag.
    const GearboxParameters& g = car.vehicle.gearbox().parameters();
    const f32 wheels = car.vehicle.wheel(0).driveTorque + car.vehicle.wheel(1).driveTorque;
    const f32 input = car.vehicle.telemetry().clutchTorque;
    const f32 measured = wheels / (car.vehicle.gearbox().ratioOf(3) * input);
    const f32 expected = g.efficiency - g.dragTorque / input;
    printf("gearbox: %.1f N m in, %.1f N m at the wheels through %.2f:1 - efficiency %.4f (set %.2f, %.4f with drag)\n",
           double(input), double(wheels), double(car.vehicle.gearbox().ratioOf(3)), double(measured),
           double(g.efficiency), double(expected));
    CHECK_NEAR(measured, expected, 0.005);
}

void automaticShiftsUp(f32* zeroToHundred)
{
    Scene scene;
    Rig car;
    PassengerCarOptions options;
    options.gearbox = GearboxMode::Automatic;
    car.layout = buildPassengerCar(car.vehicle, car.body, options);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 1.0f;
    car.vehicle.setControls(c);
    s32 highest = 0;
    bool monotone = true;
    s32 sequence[16];
    u32 shifts = 0;
    f32 hundred = -1.0f;
    for (int i = 0; i < static_cast<int>(30.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        const s32 gear = car.vehicle.gearbox().targetGear();
        if (gear < highest)
            monotone = false;
        if (gear > highest)
        {
            highest = gear;
            if (shifts < 16)
                sequence[shifts++] = gear;
        }
        if (hundred < 0.0f && car.vehicle.telemetry().forwardSpeed >= 100.0f / 3.6f)
            hundred = (i + 1) * kStep;
    }
    printf("automatic at full throttle: gears");
    for (u32 i = 0; i < shifts; ++i)
        printf(" %d", sequence[i]);
    printf(", 0-100 km/h in %.2f s, %.1f m/s after 30 s\n", double(hundred),
           double(car.vehicle.telemetry().forwardSpeed));
    CHECK(monotone);
    CHECK(highest >= 4);
    *zeroToHundred = hundred;
}

void zeroToHundred()
{
    f32 automatic = -1.0f;
    automaticShiftsUp(&automatic);
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 1.0f;
    car.vehicle.setControls(c);
    f32 manual = -1.0f;
    for (int i = 0; i < static_cast<int>(20.0f / kStep) && manual < 0.0f; ++i)
    {
        step(scene, car, kStep);
        const VehicleTelemetry& t = car.vehicle.telemetry();
        if (t.engineRpm > 6300.0f && t.gear > 0 && !t.shifting)
            car.vehicle.shiftUp();
        if (t.forwardSpeed >= 100.0f / 3.6f)
            manual = (i + 1) * kStep;
    }
    printf("0-100 km/h: %.2f s manual, %.2f s automatic (1400 kg, 100 kW, front-wheel drive)\n", double(manual),
           double(automatic));
    // A 100 kW, 1.4 t front-driver: 9 to 13 s.
    CHECK(manual > 8.0f && manual < 13.5f);
    CHECK(automatic > 8.0f && automatic < 14.0f);
}

void engineBraking()
{
    f32 decel[2];
    for (u32 inGear = 0; inGear < 2; ++inGear)
    {
        Scene scene;
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.place(scene, KM::vec3(0.0f));
        run(scene, car, 1.0f);
        car.launch(25.0f);
        car.vehicle.selectGear(inGear ? 3 : 0);
        run(scene, car, 1.0f);
        const f32 before = car.vehicle.telemetry().forwardSpeed;
        run(scene, car, 3.0f);
        decel[inGear] = (before - car.vehicle.telemetry().forwardSpeed) / 3.0f;
    }
    printf("coasting from 25 m/s: %.3f m/s^2 in neutral, %.3f m/s^2 in third (engine braking)\n", double(decel[0]),
           double(decel[1]));
    CHECK(decel[1] > decel[0] + 0.1f);
    CHECK(decel[0] > 0.1f); // rolling resistance and drag
}

void reverse()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(-1);
    VehicleControls c;
    c.throttle = 0.3f;
    car.vehicle.setControls(c);
    run(scene, car, 4.0f);
    printf("reverse: %.2f m/s, gear %d\n", double(car.vehicle.telemetry().forwardSpeed), car.vehicle.telemetry().gear);
    CHECK(car.vehicle.telemetry().gear == -1);
    CHECK(car.vehicle.telemetry().forwardSpeed < -1.0f);
}

void shiftTakesTime()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    car.vehicle.selectGear(1);
    run(scene, car, 0.5f);
    car.vehicle.shiftUp();
    int neutral = 0;
    for (int i = 0; i < 60; ++i)
    {
        step(scene, car, kStep);
        if (car.vehicle.gearbox().gear() == 0)
            ++neutral;
    }
    const f32 expected = car.vehicle.gearbox().parameters().shiftTime;
    printf("shift 1-2: %.3f s in neutral (set %.3f)\n", double(neutral * kStep), double(expected));
    CHECK_NEAR(neutral * kStep, expected, kStep + 1.0e-4);
    CHECK(car.vehicle.gearbox().gear() == 2);
}

// 300:1 reduction from a 1.5 kg m^2 flywheel to 80 kg m^2 wheels at 60 Hz: engine, gearbox and every wheel must agree.
void lockedHighRatioDriveline()
{
    Scene scene;
    Rig tractor;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    SurfaceInfo soil;
    soil.type = SurfaceType::Soil;
    tractor.vehicle.setDefaultSurface(soil);
    tractor.place(scene, KM::vec3(0.0f));
    run(scene, tractor, 1.0f);
    tractor.vehicle.setDifferentialLock(static_cast<u32>(tractor.layout.rearDifferential), true);
    tractor.vehicle.setDifferentialLock(static_cast<u32>(tractor.layout.frontDifferential), true);
    tractor.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 0.6f;
    tractor.vehicle.setControls(c);
    run(scene, tractor, 4.0f);
    const f32 overall = tractor.vehicle.gearbox().ratioOf(1);
    f32 rpmMin = 1.0e9f;
    f32 rpmMax = 0.0f;
    f32 worstMismatch = 0.0f;
    for (int i = 0; i < static_cast<int>(3.0f / kStep); ++i)
    {
        step(scene, tractor, kStep);
        const f32 rpm = tractor.vehicle.telemetry().engineRpm;
        rpmMin = fminf(rpmMin, rpm);
        rpmMax = fmaxf(rpmMax, rpm);
        const f32 engine = tractor.vehicle.engine().angularVelocity();
        const f32 rear = tractor.vehicle.wheel(0).angularVelocity;
        worstMismatch = fmaxf(worstMismatch, fabsf(engine / overall - rear));
        worstMismatch = fmaxf(worstMismatch, fabsf(tractor.vehicle.wheel(1).angularVelocity - rear));
        // Fronts geared to keep pace: same ground speed.
        const f32 frontGround =
            tractor.vehicle.wheel(2).angularVelocity * tractor.vehicle.axle(1).wheel.radius;
        worstMismatch = fmaxf(worstMismatch, fabsf(frontGround - rear * tractor.vehicle.axle(0).wheel.radius));
    }
    printf("tractor locked at %.0f:1: %.0f-%.0f rpm over 3 s, worst shaft mismatch %.2e rad/s, %.3f m/s\n",
           double(overall), double(rpmMin), double(rpmMax), double(worstMismatch),
           double(tractor.vehicle.telemetry().forwardSpeed));
    CHECK(overall > 290.0f);
    CHECK(tractor.vehicle.telemetry().clutchLocked);
    CHECK(rpmMax - rpmMin < 10.0f);
    CHECK(worstMismatch < 1.0e-3f);
    CHECK(tractor.vehicle.telemetry().forwardSpeed > 0.5f);
}
}

int main()
{
    engineControl();
    carTopSpeeds();
    tractorTopSpeeds();
    clutchLaunchAndStall();
    noChatterAtLock();
    zeroToHundred();
    engineBraking();
    reverse();
    shiftTakesTime();
    lockedHighRatioDriveline();
    return TEST_RESULT();
}
