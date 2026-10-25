#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
constexpr f32 kStep = 1.0f / 60.0f;

struct Rig2
{
    Scene scene;
    Rig truck;
    Rig trailer;
    TrailerCoupling coupling;
    CouplingParameters parameters;
    f32 worstSeparation = 0.0f;

    Rig2()
    {
        truck.layout = buildTruck6x4(truck.vehicle, truck.body);
        trailer.layout = buildSemiTrailer(trailer.vehicle, trailer.body);
        truck.place(scene, KM::vec3(0.0f));
        parameters.type = CouplingType::FifthWheel;
        parameters.towPoint = truck.layout.towPoint;
        parameters.trailerPoint = trailer.layout.couplingPoint;
        TrailerCoupling::placeTrailer(truck.body, trailer.body, parameters);
        trailer.addToWorld(scene);
        CHECK(coupling.attach(scene.world, truck.vehicle, trailer.vehicle, parameters));
    }

    void step()
    {
        Rig* rigs[2] = {&truck, &trailer};
        stepAll(scene, rigs, 2, kStep);
        worstSeparation = fmaxf(worstSeparation, coupling.separation());
    }

    void launch(f32 speed)
    {
        truck.launch(speed);
        trailer.launch(speed);
        truck.vehicle.selectGear(1);
    }

    void run(f32 seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds / kStep + 0.5f); ++i)
            step();
    }

    void cruise(f32 speed, f32 steering, f32 seconds)
    {
        VehicleControls c;
        c.steering = steering;
        for (int i = 0; i < static_cast<int>(seconds / kStep + 0.5f); ++i)
        {
            const f32 v = truck.vehicle.telemetry().forwardSpeed;
            c.throttle = kinetix::Clamp(0.3f + 0.4f * (speed - v), 0.0f, 1.0f);
            truck.vehicle.setControls(c);
            step();
        }
    }
};

void coupledAtRest()
{
    Rig2 r;
    r.run(3.0f);
    f32 lightest = 1.0e9f;
    for (u32 w = 0; w < r.truck.vehicle.wheelCount(); ++w)
        lightest = fminf(lightest, r.truck.vehicle.wheel(w).load);
    for (u32 w = 0; w < r.trailer.vehicle.wheelCount(); ++w)
        lightest = fminf(lightest, r.trailer.vehicle.wheel(w).load);
    f32 kingpin = 0.0f;
    for (u32 w = 0; w < r.trailer.vehicle.wheelCount(); ++w)
        kingpin += r.trailer.vehicle.wheel(w).load;
    kingpin = r.trailer.layout.mass * kG - kingpin;
    printf("coupled at rest: separation %.2f mm, trailer pitch %.2f deg, truck pitch %.2f deg, kingpin load %.1f kN "
           "(%.0f%% of the trailer), lightest wheel %.1f kN\n",
           double(1000.0f * r.coupling.separation()), double(r.trailer.vehicle.telemetry().pitch / kDeg),
           double(r.truck.vehicle.telemetry().pitch / kDeg), double(kingpin / 1000.0f),
           double(100.0f * kingpin / (r.trailer.layout.mass * kG)), double(lightest / 1000.0f));
    CHECK(r.worstSeparation < 0.003f);
    CHECK(fabsf(r.trailer.vehicle.telemetry().pitch) < 2.0f * kDeg);
    CHECK(lightest > 5000.0f);
    CHECK_NEAR(kingpin / (r.trailer.layout.mass * kG), 0.4, 0.05);
    CHECK(r.trailer.vehicle.brakeSource() == &r.truck.vehicle);
}

void steadyCircle()
{
    Rig2 r;
    r.run(1.0f);
    r.launch(6.0f);
    r.cruise(6.0f, 0.0f, 3.0f);
    r.cruise(6.0f, 0.4f, 20.0f);
    f32 lo = 1.0e9f;
    f32 hi = -1.0e9f;
    ct::Vector<KM::vec3> kingpinTrack;
    f32 lowestUp = 1.0f;
    for (int i = 0; i < static_cast<int>(12.0f / kStep); ++i)
    {
        r.cruise(6.0f, 0.4f, kStep);
        const f32 a = r.coupling.articulationAngle();
        lo = fminf(lo, a);
        hi = fmaxf(hi, a);
        kingpinTrack.push_back(r.coupling.towPointWorld());
        lowestUp = fminf(lowestUp, fminf(r.truck.lowestUp(), r.trailer.lowestUp()));
    }
    f32 cx, cz, radius;
    fitCircle(kingpinTrack, cx, cz, radius);
    const f32 trailerBase = r.trailer.layout.couplingPoint.z - r.trailer.vehicle.referenceAxlePosition();
    // A trailer following its kingpin round a circle of radius R points
    // asin(L / R) outside the kingpin's direction of travel; the kingpin,
    // ahead of the tandem by d, travels turned asin(d / R) into the turn
    // from the tractor's heading.
    const f32 offset = r.parameters.towPoint.z - r.truck.vehicle.referenceAxlePosition();
    const f32 theory = asinf(trailerBase / radius) - asinf(offset / radius);
    const f32 measured = -0.5f * (lo + hi);
    printf("circle at %.2f m/s: kingpin radius %.2f m, articulation %.2f deg (%.2f..%.2f), kinematic %.2f deg "
           "(%+.1f%%), worst separation %.2f mm, lowest up %.3f, gear %d\n",
           double(r.truck.vehicle.telemetry().forwardSpeed), double(radius), double(measured / kDeg),
           double(-hi / kDeg), double(-lo / kDeg), double(theory / kDeg),
           double(100.0f * (measured - theory) / theory), double(1000.0f * r.worstSeparation), double(lowestUp),
           r.truck.vehicle.telemetry().gear);
    // Turning right, the trailer swings to the tow's left.
    CHECK(hi < 0.0f);
    CHECK(hi - lo < 0.5f * kDeg);
    CHECK_NEAR(measured, theory, 0.05 * theory);
    CHECK(fabsf(lo) < 45.0f * kDeg);
    CHECK(r.worstSeparation < 0.005f);
    CHECK(lowestUp > 0.95f);
    CHECK(finiteVec(r.truck.body.position()) && finiteVec(r.trailer.body.position()));
}

f32 brakeFromSpeed(bool linked, f32* articulation, f32* trailerBrake)
{
    Rig2 r;
    if (!linked)
        r.trailer.vehicle.setBrakeSource(nullptr);
    r.run(1.0f);
    r.launch(10.0f);
    r.cruise(10.0f, 0.0f, 3.0f);
    const s32 startGear = r.truck.vehicle.telemetry().gear;
    CHECK(startGear >= 8);
    CHECK_NEAR(r.truck.vehicle.telemetry().forwardSpeed, 10.0, 0.5);
    const KM::vec3 start = r.truck.body.position();
    VehicleControls c;
    c.brake = 0.5f;
    r.truck.vehicle.setControls(c);
    *articulation = 0.0f;
    *trailerBrake = 0.0f;
    for (int i = 0; i < static_cast<int>(12.0f / kStep); ++i)
    {
        r.step();
        *articulation = fmaxf(*articulation, fabsf(r.coupling.articulationAngle()));
        if (i == 30)
            *trailerBrake = r.trailer.vehicle.appliedBrake();
    }
    CHECK(fabsf(r.truck.vehicle.telemetry().forwardSpeed) < 0.05f);
    CHECK(r.worstSeparation < 0.005f);
    printf("  braking %s: gear %d before braking, after: %.3f m/s gear %d rpm %.0f clutch %.2f\n", linked ? "linked" : "unlinked", startGear, double(r.truck.vehicle.telemetry().forwardSpeed), r.truck.vehicle.telemetry().gear, double(r.truck.vehicle.telemetry().engineRpm), double(r.truck.vehicle.telemetry().clutchEngagement));
    if (linked)
    {
        for (u32 w = 0; w < r.trailer.vehicle.wheelCount(); ++w)
            CHECK_NEAR(r.trailer.vehicle.wheel(w).brakeTorque, 0.5f * r.trailer.vehicle.axle(w / 2).wheel.maxBrakeTorque,
                       1.0f);
    }
    return horizontalDistance(r.truck.body.position(), start);
}

void trailerBrakesFollow()
{
    f32 linkedArticulation, freeArticulation, linkedBrake, freeBrake;
    const f32 linked = brakeFromSpeed(true, &linkedArticulation, &linkedBrake);
    const f32 free = brakeFromSpeed(false, &freeArticulation, &freeBrake);
    printf("half brake from 10 m/s: %.1f m with the trailer brakes on the line (trailer brake input %.2f, "
           "articulation <= %.2f deg), %.1f m with the tractor braking alone (%.2f)\n",
           double(linked), double(linkedBrake), double(linkedArticulation / kDeg), double(free), double(freeBrake));
    CHECK_NEAR(linkedBrake, 0.5, 1.0e-6);
    CHECK(freeBrake == 0.0f);
    CHECK(linked < 0.8f * free);
    CHECK(linkedArticulation < 1.0f * kDeg);
}

void detachAndReattach()
{
    Rig2 r;
    r.run(0.5f);
    CHECK(r.scene.world.jointCount() == 1);
    CHECK(r.scene.world.pairIgnored(r.truck.id, r.trailer.id));
    r.coupling.detach();
    CHECK(!r.coupling.attached());
    CHECK(r.scene.world.jointCount() == 0);
    CHECK(!r.scene.world.pairIgnored(r.truck.id, r.trailer.id));
    CHECK(r.trailer.vehicle.brakeSource() == nullptr);
    CHECK(r.coupling.separation() == 0.0f);
}
}

int main()
{
    coupledAtRest();
    steadyCircle();
    trailerBrakesFollow();
    detachAndReattach();
    return TEST_RESULT();
}
