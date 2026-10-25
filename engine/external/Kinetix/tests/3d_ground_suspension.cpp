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

void staticSag()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 3.0f);
    f32 total = 0.0f;
    for (u32 w = 0; w < 4; ++w)
    {
        const WheelTelemetry& t = car.vehicle.wheel(w);
        const SuspensionParameters& s = car.vehicle.axle(w / 2).wheel.suspension;
        total += t.load;
        CHECK_NEAR(t.compression, s.staticCompression(t.load), 1.0e-3);
        CHECK_NEAR(t.suspensionForce, t.load, 0.01 * t.load);
    }
    printf("static: body at %.4f m (designed %.3f), loads %.0f %.0f %.0f %.0f N, sum %.0f N (weight %.0f)\n",
           double(car.body.position().y), double(car.layout.restHeight), double(car.vehicle.wheel(0).load),
           double(car.vehicle.wheel(1).load), double(car.vehicle.wheel(2).load), double(car.vehicle.wheel(3).load),
           double(total), double(car.layout.mass * kG));
    CHECK_NEAR(total, car.layout.mass * kG, 0.002 * car.layout.mass * kG);
    CHECK_NEAR(car.body.position().y, car.layout.restHeight, 0.005);
    CHECK_NEAR(car.vehicle.wheel(0).load, 0.6f * car.layout.mass * kG / 2.0f, 0.01 * car.layout.mass * kG);
    CHECK(fabsf(car.vehicle.telemetry().pitch) < 0.05f * kDeg);
}

void heave()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 2.0f);
    const f32 rest = car.body.position().y;
    car.body.setPosition(car.body.position() + KM::vec3(0.0f, 0.04f, 0.0f));
    car.body.calculateDerivedData();
    f32 previous = 0.04f;
    f32 crossings[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    u32 count = 0;
    f32 lowest = 1.0f;
    for (int i = 0; i < static_cast<int>(4.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        const f32 offset = car.body.position().y - rest;
        lowest = fminf(lowest, offset);
        if (count < 4 && ((previous > 0.0f) != (offset > 0.0f)))
        {
            crossings[count++] = (static_cast<f32>(i) + previous / (previous - offset)) * kStep;
        }
        previous = offset;
    }
    const f32 period = count >= 3 ? (crossings[2] - crossings[0]) : 0.0f;
    // The springs' rate over the sprung mass, ignoring pitch coupling.
    f32 stiffness = 0.0f;
    for (u32 a = 0; a < 2; ++a)
        stiffness += 2.0f * car.vehicle.axle(a).wheel.suspension.stiffness;
    const f32 theory = 2.0f * kPi * sqrtf(car.layout.mass / stiffness);
    printf("heave: period %.3f s (sqrt(m / k) gives %.3f s), first undershoot %.1f mm, %.2f mm off rest after 4 s\n",
           double(period), double(theory), double(1000.0f * lowest),
           double(1000.0f * (car.body.position().y - rest)));
    CHECK(count >= 3);
    CHECK(period > 0.95f * theory && period < 1.3f * theory);
    CHECK(lowest > -0.03f);
    CHECK(fabsf(car.body.position().y - rest) < 0.002f);
}

void dropOntoBumpStops()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f, 0.3f, 0.0f));
    f32 deepest = 0.0f;
    f32 lowestBody = 1.0e9f;
    f32 strongest = 0.0f;
    for (int i = 0; i < static_cast<int>(4.0f / kStep); ++i)
    {
        step(scene, car, kStep);
        for (u32 w = 0; w < 4; ++w)
        {
            deepest = fmaxf(deepest, car.vehicle.wheel(w).compression);
            strongest = fmaxf(strongest, car.vehicle.wheel(w).load);
        }
        lowestBody = fminf(lowestBody, car.body.position().y - car.layout.collisionHalfExtents.y);
    }
    const SuspensionParameters& s = car.vehicle.axle(0).wheel.suspension;
    printf("drop from 0.3 m: deepest compression %.3f of %.3f m travel (bump stops from %.3f), peak wheel load "
           "%.1f kN, lowest underbody %.3f m, settled at %.4f m\n",
           double(deepest), double(s.travel), double(s.travel - s.bumpStopRange), double(strongest / 1000.0f),
           double(lowestBody), double(car.body.position().y));
    CHECK(deepest > s.travel - s.bumpStopRange);
    CHECK(deepest <= s.travel + 1.0e-4f);
    CHECK(lowestBody > 0.02f);
    CHECK_NEAR(car.body.position().y, car.layout.restHeight, 0.005);
}

void pastFullBump()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f));
    run(scene, car, 1.0f);
    // 4 tonnes pressing down through the centre of mass for two seconds.
    f32 lowest = 1.0e9f;
    for (int i = 0; i < static_cast<int>(2.0f / kStep); ++i)
    {
        car.body.addForce(KM::vec3(0.0f, -40000.0f, 0.0f));
        step(scene, car, kStep);
        lowest = fminf(lowest, car.body.position().y);
    }
    // Full bump puts the body origin at radius - hard point height.
    const f32 fullBump = car.vehicle.axle(0).wheel.radius - car.vehicle.axle(0).height;
    printf("pressed with 40 kN: body origin down to %.4f m, full bump at %.4f m (front), wheel loads %.1f kN front\n",
           double(lowest), double(fullBump), double(car.vehicle.wheel(0).load / 1000.0f));
    CHECK(lowest > fullBump - 0.02f);
    run(scene, car, 2.0f);
    CHECK_NEAR(car.body.position().y, car.layout.restHeight, 0.005);
}

void airborneWheels()
{
    Scene scene;
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f, 3.0f, 0.0f));
    car.vehicle.setWheelAngularVelocity(0, 10.0f);
    run(scene, car, 0.3f);
    for (u32 w = 0; w < 4; ++w)
    {
        CHECK(!car.vehicle.wheel(w).inContact);
        CHECK(car.vehicle.wheel(w).load == 0.0f);
        CHECK(car.vehicle.wheel(w).longitudinalForce == 0.0f);
    }
    CHECK(car.vehicle.wheel(0).angularVelocity > 9.5f && car.vehicle.wheel(0).angularVelocity < 10.0f);
}
}

int main()
{
    staticSag();
    heave();
    dropOntoBumpStops();
    pastFullBump();
    airborneWheels();
    return TEST_RESULT();
}
