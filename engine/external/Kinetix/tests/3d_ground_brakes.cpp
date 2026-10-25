#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
constexpr f32 kStep = 1.0f / 60.0f;

void settle(Scene& scene, Rig& rig, f32 seconds = 1.0f)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5f);
    for (int i = 0; i < steps; ++i)
        step(scene, rig, kStep);
}

// The friction coefficient the locked tyres slide with: the model's force
// at a slip ratio of -1 over the load, at the car's static wheel loads.
f32 lockedFriction(const Rig& car)
{
    const Tyre tyre(car.vehicle.axle(0).wheel.tyre);
    f32 sum = 0.0f;
    f32 load = 0.0f;
    for (u32 w = 0; w < 4; ++w)
    {
        const f32 fz = car.vehicle.wheel(w).load;
        sum += -tyre.evaluate(fz, -1.0f, 0.0f, SurfaceInfo()).fx;
        load += fz;
    }
    return sum / load;
}

void fullStopLocksWithoutOscillation()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    settle(scene, car);
    const f32 mu = lockedFriction(car);
    car.launch(20.0f);
    VehicleControls c;
    c.brake = 1.0f;
    car.vehicle.setControls(c);
    const KM::vec3 start = car.body.position();

    bool lockedOnce[4] = {false, false, false, false};
    bool unlockedAfter = false;
    bool reversed = false;
    bool monotone = true;
    f32 previous[4];
    for (u32 w = 0; w < 4; ++w)
        previous[w] = 20.0f / car.vehicle.axle(w / 2).wheel.radius;
    f32 lockTime = -1.0f;
    f32 stopTime = -1.0f;
    for (int i = 0; i < static_cast<int>(6.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        bool all = true;
        for (u32 w = 0; w < 4; ++w)
        {
            const WheelTelemetry& t = car.vehicle.wheel(w);
            if (t.locked)
                lockedOnce[w] = true;
            else if (lockedOnce[w] && car.vehicle.telemetry().forwardSpeed > 0.05f)
                unlockedAfter = true;
            if (t.angularVelocity < -1.0e-3f)
                reversed = true;
            if (t.angularVelocity > previous[w] + 1.0e-3f)
            {
                if (monotone)
                    printf("  wheel %u sped up at step %d: %.4f -> %.4f\n", w, i, double(previous[w]),
                           double(t.angularVelocity));
                monotone = false;
            }
            previous[w] = t.angularVelocity;
            all = all && lockedOnce[w];
        }
        if (all && lockTime < 0.0f)
            lockTime = (i + 1) * kStep;
        if (stopTime < 0.0f && car.vehicle.telemetry().forwardSpeed < 0.01f)
            stopTime = (i + 1) * kStep;
    }
    const f32 distance = horizontalDistance(car.body.position(), start);
    const f32 theory = 20.0f * 20.0f / (2.0f * mu * kG);
    printf("full stop from 20 m/s: all wheels locked after %.2f s, stopped after %.2f s in %.2f m; "
           "v^2/(2 mu g) with locked mu %.3f = %.2f m (%+.1f%%)\n",
           double(lockTime), double(stopTime), double(distance), double(mu), double(theory),
           double(100.0f * (distance - theory) / theory));
    for (u32 w = 0; w < 4; ++w)
        CHECK(lockedOnce[w]);
    CHECK(lockTime > 0.0f && lockTime < 0.5f);
    CHECK(!unlockedAfter);
    CHECK(!reversed);
    CHECK(monotone);
    CHECK_NEAR(distance, theory, 0.1 * theory);
    const KM::vec3 stopped = car.body.position();
    settle(scene, car, 3.0f);
    CHECK(horizontalDistance(car.body.position(), stopped) < 0.005f);
    for (u32 w = 0; w < 4; ++w)
        CHECK(fabsf(car.vehicle.wheel(w).angularVelocity) < 1.0e-3f);
}

void handbrakeLocksOnlyTheRear()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    settle(scene, car);
    car.launch(15.0f);
    VehicleControls c;
    c.handbrake = 1.0f;
    car.vehicle.setControls(c);
    for (int i = 0; i < 60; ++i)
        step(scene, car, kStep);
    const f32 v = car.vehicle.telemetry().forwardSpeed;
    const f32 r = car.vehicle.axle(0).wheel.radius;
    printf("handbrake at %.1f m/s: front omega %.2f %.2f (rolling %.2f), rear %.4f %.4f, locked %d%d%d%d\n",
           double(v), double(car.vehicle.wheel(0).angularVelocity), double(car.vehicle.wheel(1).angularVelocity),
           double(v / r), double(car.vehicle.wheel(2).angularVelocity), double(car.vehicle.wheel(3).angularVelocity),
           car.vehicle.wheel(0).locked, car.vehicle.wheel(1).locked, car.vehicle.wheel(2).locked,
           car.vehicle.wheel(3).locked);
    CHECK(car.vehicle.wheel(2).locked && car.vehicle.wheel(3).locked);
    CHECK(!car.vehicle.wheel(0).locked && !car.vehicle.wheel(1).locked);
    CHECK_NEAR(car.vehicle.wheel(0).angularVelocity, v / r, 0.05 * v / r);
    CHECK_NEAR(car.vehicle.wheel(1).angularVelocity, v / r, 0.05 * v / r);
    CHECK(fabsf(car.vehicle.wheel(2).angularVelocity) < 1.0e-3f);
    CHECK(v < 15.0f && v > 8.0f);
}

void antiLockKeepsSteering(bool antiLock, f32* distance, f32* heading, f32* lockedFraction)
{
    Scene scene;
    Rig car;
    PassengerCarOptions options;
    options.antiLock = antiLock;
    car.layout = buildPassengerCar(car.vehicle, car.body, options);
    car.place(scene, KM::vec3(0.0f));
    settle(scene, car);
    car.launch(20.0f);
    VehicleControls c;
    c.brake = 1.0f;
    c.steering = 0.25f;
    car.vehicle.setControls(c);
    const KM::vec3 start = car.body.position();
    int lockedSteps = 0;
    int movingSteps = 0;
    for (int i = 0; i < static_cast<int>(6.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        if (car.vehicle.telemetry().forwardSpeed > 3.0f)
        {
            ++movingSteps;
            if (car.vehicle.wheel(0).locked || car.vehicle.wheel(1).locked)
                ++lockedSteps;
        }
    }
    *distance = horizontalDistance(car.body.position(), start);
    *heading = car.vehicle.telemetry().heading;
    *lockedFraction = movingSteps > 0 ? static_cast<f32>(lockedSteps) / movingSteps : 0.0f;
}

void antiLock()
{
    f32 lockedDistance, lockedHeading, lockedFraction;
    f32 absDistance, absHeading, absFraction;
    antiLockKeepsSteering(false, &lockedDistance, &lockedHeading, &lockedFraction);
    antiLockKeepsSteering(true, &absDistance, &absHeading, &absFraction);
    printf("braking with steering: locked %.1f m, heading %.1f deg, fronts locked %.0f%% of the time; "
           "anti-lock %.1f m, heading %.1f deg, %.0f%%\n",
           double(lockedDistance), double(lockedHeading / kDeg), double(100.0f * lockedFraction),
           double(absDistance), double(absHeading / kDeg), double(100.0f * absFraction));
    CHECK(lockedFraction > 0.9f);
    CHECK(absFraction < 0.1f);
    // Locked fronts plough straight on (a few degrees from the 0.2 s before they lock; a locked car's slide is yaw-unstable so the
    // exact figure is not), while with anti-lock the car turns.
    CHECK(fabsf(lockedHeading) < 15.0f * kDeg);
    CHECK(absHeading > 30.0f * kDeg);
    CHECK(absHeading > 3.0f * fabsf(lockedHeading));
    CHECK(absDistance < 1.1f * lockedDistance);
}

void parkedOnSlope(f32 heading)
{
    Scene scene(false);
    const f32 angle = atanf(0.2f);
    const KM::quat slope = KM::angleAxis(-angle, KM::vec3(1.0f, 0.0f, 0.0f));
    const KM::vec3 normal = slope * KM::vec3(0.0f, 1.0f, 0.0f);
    scene.addStaticBox(-normal * 1.0f, KM::vec3(100.0f, 1.0f, 100.0f), slope);
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.body.setPosition(normal * car.layout.restHeight);
    car.body.setOrientation(slope * headingRotation(heading));
    car.body.calculateDerivedData();
    car.addToWorld(scene);
    VehicleControls c;
    c.handbrake = 1.0f;
    car.vehicle.setControls(c);
    settle(scene, car, 1.0f);
    const KM::vec3 parked = car.body.position();
    f32 worst = 0.0f;
    for (int i = 0; i < static_cast<int>(20.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        worst = fmaxf(worst, KM::length(car.body.position() - parked));
    }
    printf("parked on 20%% facing %s: moved %.2f mm in 20 s (%.2f mm settling), rear Fx %.0f %.0f N\n",
           heading == 0.0f ? "uphill" : "downhill", double(1000.0f * worst),
           double(1000.0f * KM::length(parked - normal * car.layout.restHeight)),
           double(car.vehicle.wheel(2).longitudinalForce), double(car.vehicle.wheel(3).longitudinalForce));
    CHECK(worst < 0.01f);
    CHECK(KM::length(parked - normal * car.layout.restHeight) < 0.05f);
}

void finiteAtStandstill()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    settle(scene, car, 2.0f);
    bool finite = true;
    for (u32 w = 0; w < 4; ++w)
    {
        const WheelTelemetry& t = car.vehicle.wheel(w);
        finite = finite && isfinite(t.slipRatio) && isfinite(t.slipAngle) && isfinite(t.kinematicSlipRatio) &&
                 isfinite(t.kinematicSlipAngle) && isfinite(t.longitudinalForce) && isfinite(t.lateralForce) &&
                 isfinite(t.angularVelocity);
        CHECK(t.inContact);
        CHECK(fabsf(t.angularVelocity) < 1.0e-3f);
        CHECK(fabsf(t.longitudinalForce) < 5.0f);
    }
    CHECK(finite);
    car.vehicle.selectGear(1);
    VehicleControls c;
    c.throttle = 1.0f;
    car.vehicle.setControls(c);
    for (int i = 0; i < 60; ++i)
    {
        step(scene, car, kStep);
        for (u32 w = 0; w < 4; ++w)
            finite = finite && isfinite(car.vehicle.wheel(w).slipRatio) &&
                     isfinite(car.vehicle.wheel(w).longitudinalForce);
    }
    CHECK(finite);
    CHECK(car.vehicle.telemetry().forwardSpeed > 0.1f);
}

void partialBrakeDoesNotLock()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    settle(scene, car);
    car.launch(20.0f);
    VehicleControls c;
    c.brake = 0.3f;
    car.vehicle.setControls(c);
    bool locked = false;
    for (int i = 0; i < 60; ++i)
    {
        step(scene, car, kStep);
        for (u32 w = 0; w < 4; ++w)
            locked = locked || car.vehicle.wheel(w).locked;
    }
    const f32 v = car.vehicle.telemetry().forwardSpeed;
    // 30% of the brakes: roughly 0.3 * (2 * 1800 + 2 * 900) / r / m.
    const f32 expected = 0.3f * (2.0f * 1800.0f + 2.0f * 900.0f) / car.vehicle.axle(0).wheel.radius / car.layout.mass;
    printf("30%% brake: %.2f m/s after 1 s (%.2f m/s^2, brake torque alone %.2f), locked %d\n", double(v),
           double(20.0f - v), double(expected), locked);
    CHECK(!locked);
    CHECK_NEAR(20.0f - v, expected, 0.12 * expected);
}
}

int main()
{
    fullStopLocksWithoutOscillation();
    handbrakeLocksOnlyTheRear();
    antiLock();
    parkedOnSlope(0.0f);
    parkedOnSlope(kPi);
    finiteAtStandstill();
    partialBrakeDoesNotLock();
    return TEST_RESULT();
}
