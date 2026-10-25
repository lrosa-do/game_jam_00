#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
void minute(bool bumpy, f32 rate)
{
    Scene scene(!bumpy);
    if (bumpy)
        scene.addStatic(makeHeightfield(200, 1.0f, bumpyHeight), KM::vec3(0.0f), KM::quat());
    Rig car;
    car.layout = buildPassengerCar(car.vehicle, car.body);
    car.place(scene, KM::vec3(0.0f, bumpy ? bumpyHeight(0.0f, 0.0f) : 0.0f, 0.0f));
    Rig* rigs[1] = {&car};
    StabilityRun r;
    stabilityDrive(scene, rigs, 1, nullptr, rate, 9.0f, 0.35f, 2, r);
    reportStability("car", rate, bumpy, r);
    CHECK(r.finite);
    CHECK(r.lowestUp > 0.9f);
    CHECK(r.fastest < 20.0f);
    CHECK(r.fastestWheel < 100.0f);
    CHECK(r.farthest < 75.0f);
    CHECK(r.stoppedSpeed < 0.05f);
}

void highSpeedHandling(bool largeTrackMesh)
{
    const f32 speeds[] = {190.0f / 3.6f, 200.0f / 3.6f};
    for (f32 speed : speeds)
    {
        Scene scene(!largeTrackMesh);
        if (largeTrackMesh)
        {
            scene.addStatic(makeHeightfield(2, 2000.0f, [](f32, f32) { return 0.0f; }), KM::vec3(0.0f),
                            KM::quat());
        }
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.place(scene, KM::vec3(0.0f));
        car.vehicle.selectGear(0);
        car.launch(speed);

        const f32 dt = 1.0f / 60.0f;
        f32 lowestUp = 1.0f;
        f32 lateralTravel = 0.0f;
        u32 unsupportedSteps = 0;
        bool finite = true;
        for (int i = 0; i < 5 * 60; ++i)
        {
            VehicleControls controls;
            controls.steering = i >= 60 ? 0.002f : 0.0f;
            car.vehicle.setControls(controls);
            step(scene, car, dt);

            const KM::vec3 position = car.body.position();
            const KM::vec3 velocity = car.body.velocity();
            const VehicleTelemetry& telemetry = car.vehicle.telemetry();
            finite = finite && finiteVec(position) && finiteVec(velocity) &&
                     finiteVec(car.body.angularVelocity()) && isfinite(telemetry.forwardSpeed);
            lowestUp = fminf(lowestUp, car.lowestUp());
            lateralTravel = fmaxf(lateralTravel, fabsf(position.x));
            u32 groundedWheels = 0;
            for (u32 wheel = 0; wheel < car.vehicle.wheelCount(); ++wheel)
                groundedWheels += car.vehicle.wheel(wheel).inContact ? 1u : 0u;
            unsupportedSteps += groundedWheels < 2 ? 1u : 0u;
        }

        const f32 finalSpeed = car.vehicle.telemetry().forwardSpeed;
        printf("car on %s at %.0f km/h: final %.1f km/h, lowest up %.3f, lateral travel %.1f m, "
               "unsupported steps %u, finite %d\n",
               largeTrackMesh ? "large track mesh" : "box floor", double(speed * 3.6f),
               double(finalSpeed * 3.6f), double(lowestUp), double(lateralTravel), unsupportedSteps, finite);
        CHECK(finite);
        CHECK(lowestUp > 0.9f);
        CHECK(finalSpeed > 40.0f);
        CHECK(lateralTravel > 1.0f);
        CHECK(unsupportedSteps < 30);
    }
}

void stepCallbackTiming()
{
    KM::vec3 positions[2];
    KM::quat orientations[2];
    for (u32 mode = 0; mode < 2; ++mode)
    {
        Scene scene;
        scene.world.setFixedStep(1.0f / 60.0f);
        Rig car;
        car.layout = buildPassengerCar(car.vehicle, car.body);
        car.place(scene, KM::vec3(0.0f));
        car.vehicle.selectGear(1);
        VehicleControls c;
        c.throttle = 0.6f;
        c.steering = 0.3f;
        car.vehicle.setControls(c);
        if (mode == 0)
        {
            for (int i = 0; i < 300; ++i)
            {
                car.vehicle.applyForces(1.0f / 60.0f);
                scene.world.step(1.0f / 60.0f);
            }
        }
        else
        {
            scene.world.setStepCallback(GroundVehicle::stepCallback, &car.vehicle);
            car.vehicle.applyForces(1.0f / 60.0f);
            for (int i = 0; i < 100; ++i)
                scene.world.update(3.0f / 60.0f + 1.0e-6f);
        }
        positions[mode] = car.body.position();
        orientations[mode] = car.body.orientation();
    }
    printf("step callback: hand-driven (%.6f, %.6f, %.6f), callback (%.6f, %.6f, %.6f)\n", double(positions[0].x),
           double(positions[0].y), double(positions[0].z), double(positions[1].x), double(positions[1].y),
           double(positions[1].z));
    CHECK(positions[0].x == positions[1].x && positions[0].y == positions[1].y && positions[0].z == positions[1].z);
    CHECK(orientations[0].x == orientations[1].x && orientations[0].w == orientations[1].w);
    CHECK(horizontalDistance(positions[0], KM::vec3(0.0f)) > 5.0f);
}
}

int main()
{
    minute(false, 60.0f);
    minute(false, 120.0f);
    minute(true, 60.0f);
    minute(true, 120.0f);
    highSpeedHandling(false);
    highSpeedHandling(true);
    stepCallbackTiming();
    return TEST_RESULT();
}
