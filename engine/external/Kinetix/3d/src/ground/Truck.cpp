#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/GroundVehiclePresets.h"

#include "PresetSupport.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kTruckRestHeight = 1.1f;
constexpr f32 kTruckRadius = 0.53f;
constexpr f32 kTandemSpread = 1.35f;

void truckDrivetrain(GroundVehicle& vehicle, GearboxMode mode)
{
    EngineParameters engine =
        EngineParameters::diesel(300000.0f, 1900.0f, 2000.0f, 1100.0f, 1400.0f, 600.0f, 2150.0f);
    engine.inertia = 3.5f;
    engine.frictionTorque = 60.0f;
    engine.frictionPerRpm = 0.05f;
    engine.idleGain = 3.0f;
    engine.maxIdleThrottle = 0.3f;
    engine.starterTorque = 400.0f;
    engine.starterCatchRpm = 250.0f;
    vehicle.engine().setParameters(engine);
    vehicle.engine().start();

    ClutchParameters clutch;
    clutch.maxTorque = 3200.0f;
    vehicle.clutch().setParameters(clutch);

    GearboxParameters gearbox;
    const f32 ratios[12] = {14.93f, 11.64f, 9.02f, 7.04f, 5.64f, 4.40f, 3.39f, 2.65f, 2.05f, 1.60f, 1.24f, 1.0f};
    gearbox.setRatios(ratios, 12);
    gearbox.reverseRatio = 14.0f;
    gearbox.finalDrive = 3.42f;
    gearbox.efficiency = 0.95f;
    gearbox.dragTorque = 8.0f;
    gearbox.inputInertia = 0.3f;
    gearbox.shiftTime = 0.4f;
    gearbox.upshiftRpmLight = 1300.0f;
    gearbox.upshiftRpmFull = 1850.0f;
    gearbox.downshiftRpmLight = 850.0f;
    gearbox.downshiftRpmFull = 1150.0f;
    gearbox.minShiftInterval = 1.5f;
    vehicle.gearbox().setParameters(gearbox);
    vehicle.gearbox().setMode(mode);
    vehicle.setAutoClutch(true);
    vehicle.setAutoClutchSpeeds(700.0f, 950.0f, 1300.0f, 0.4f);
}

void truckTandem(GroundVehicle& vehicle, u32 firstAxle, u32 secondAxle, GroundVehicleLayout& layout)
{
    DifferentialParameters axle;
    axle.type = DifferentialType::Open;
    axle.lockable = true;
    axle.inputInertia = 0.6f;
    layout.rearDifferential = static_cast<s32>(
        vehicle.addDifferential(axle, DriveOutput::wheel(2 * firstAxle), DriveOutput::wheel(2 * firstAxle + 1)));
    layout.secondRearDifferential = static_cast<s32>(
        vehicle.addDifferential(axle, DriveOutput::wheel(2 * secondAxle), DriveOutput::wheel(2 * secondAxle + 1)));
    DifferentialParameters divider;
    divider.type = DifferentialType::Open;
    divider.lockable = true;
    divider.inputInertia = 0.4f;
    layout.centreDifferential = static_cast<s32>(vehicle.addDifferential(
        divider, DriveOutput::differential(static_cast<u32>(layout.rearDifferential)),
        DriveOutput::differential(static_cast<u32>(layout.secondRearDifferential))));
    vehicle.setDriveRoot(layout.centreDifferential);
}

AxleParameters truckSteerAxle(f32 position, f32 wheelLoad)
{
    AxleParameters a;
    a.position = position;
    a.halfTrack = 1.03f;
    a.steered = true;
    a.wheel.radius = kTruckRadius;
    a.wheel.width = 0.315f;
    a.wheel.inertia = 14.0f;
    a.wheel.tyre = TyreParameters::truck(false);
    a.wheel.suspension = SuspensionParameters::forLoad(wheelLoad, 1.6f, 0.3f, 0.2f, 0.55f);
    a.wheel.maxBrakeTorque = 12000.0f;
    a.wheel.bearingFriction = 6.0f;
    a.antiRollStiffness = 100000.0f;
    return a;
}

AxleParameters truckDriveAxle(f32 position, f32 designWheelLoad)
{
    AxleParameters a;
    a.position = position;
    a.halfTrack = 0.92f;
    a.wheel.radius = kTruckRadius;
    a.wheel.width = 0.63f;
    a.wheel.inertia = 30.0f;
    a.wheel.tyre = TyreParameters::truck(true);
    a.wheel.suspension = SuspensionParameters::forLoad(designWheelLoad, 1.6f, 0.3f, 0.2f, 0.6f);
    a.wheel.maxBrakeTorque = 10000.0f;
    a.wheel.maxHandbrakeTorque = 14000.0f;
    a.wheel.bearingFriction = 10.0f;
    a.antiRollStiffness = 150000.0f;
    return a;
}

GroundVehicleLayout buildTruck(GroundVehicle& vehicle, RigidBody& body, GearboxMode mode, bool eightByFour)
{
    GroundVehicleLayout layout;
    const f32 mass = eightByFour ? 12000.0f : 9000.0f;
    layout.mass = mass;
    layout.restHeight = kTruckRestHeight;
    const f32 weight = mass * preset::kGravity;

    u32 firstDrive = 0;
    u32 secondDrive = 0;
    if (!eightByFour)
    {
        const f32 wheelbase = 3.8f;
        const f32 frontShare = 5500.0f / 9000.0f;
        const f32 frontZ = (1.0f - frontShare) * wheelbase;
        const f32 tandemZ = frontZ - wheelbase;
        layout.wheelbase = wheelbase;
        layout.collisionHalfExtents = Math::vec3(1.25f, 0.6f, 3.0f);
        layout.towPoint = Math::vec3(0.0f, 1.2f - kTruckRestHeight, tandemZ + 0.3f);
        preset::setupBody(body, mass, Math::vec3(1.1f, 0.8f, 3.0f));

        const f32 frontLoad = frontShare * weight;
        const f32 driveLoad = 0.5f * (weight - frontLoad);
        AxleParameters front = truckSteerAxle(frontZ, 0.5f * frontLoad);
        front.height = preset::axleHeightFor(front, frontLoad, kTruckRestHeight);
        // Springs for a coupled semi-trailer's kingpin load; empty it rides a little high.
        AxleParameters drive1 = truckDriveAxle(tandemZ + 0.5f * kTandemSpread, 33000.0f);
        drive1.height = preset::axleHeightFor(drive1, driveLoad, kTruckRestHeight);
        AxleParameters drive2 = truckDriveAxle(tandemZ - 0.5f * kTandemSpread, 33000.0f);
        drive2.height = preset::axleHeightFor(drive2, driveLoad, kTruckRestHeight);
        vehicle.addAxle(front);
        firstDrive = vehicle.addAxle(drive1);
        secondDrive = vehicle.addAxle(drive2);
        layout.track = 2.0f * front.halfTrack;
    }
    else
    {
        const f32 groupBase = 0.95f + 3.5f;
        const f32 frontGroupZ = 0.5f * groupBase;
        const f32 tandemZ = frontGroupZ - groupBase;
        layout.wheelbase = frontGroupZ + 0.95f - tandemZ;
        layout.collisionHalfExtents = Math::vec3(1.25f, 0.6f, 4.6f);
        layout.towPoint = Math::vec3(0.0f, 0.6f - kTruckRestHeight, tandemZ - 1.6f);
        preset::setupBody(body, mass, Math::vec3(1.1f, 0.8f, 4.6f));

        const f32 axleLoad = 0.25f * weight;
        AxleParameters front1 = truckSteerAxle(frontGroupZ + 0.95f, 0.5f * axleLoad);
        front1.height = preset::axleHeightFor(front1, axleLoad, kTruckRestHeight);
        AxleParameters front2 = truckSteerAxle(frontGroupZ - 0.95f, 0.5f * axleLoad);
        front2.height = preset::axleHeightFor(front2, axleLoad, kTruckRestHeight);
        AxleParameters drive1 = truckDriveAxle(tandemZ + 0.5f * kTandemSpread, 40000.0f);
        drive1.height = preset::axleHeightFor(drive1, axleLoad, kTruckRestHeight);
        AxleParameters drive2 = truckDriveAxle(tandemZ - 0.5f * kTandemSpread, 40000.0f);
        drive2.height = preset::axleHeightFor(drive2, axleLoad, kTruckRestHeight);
        vehicle.addAxle(front1);
        vehicle.addAxle(front2);
        firstDrive = vehicle.addAxle(drive1);
        secondDrive = vehicle.addAxle(drive2);
        layout.track = 2.0f * front1.halfTrack;
    }

    SteeringParameters steering;
    steering.maxAngle = 0.55f;
    steering.rate = 0.8f;
    steering.ackermann = 1.0f;
    vehicle.setSteering(steering);

    truckDrivetrain(vehicle, mode);
    truckTandem(vehicle, firstDrive, secondDrive, layout);
    vehicle.setAerodynamics(5.4f, Math::vec3(0.0f, 0.8f, 0.5f));
    return layout;
}

} // namespace

GroundVehicleLayout buildTruck6x4(GroundVehicle& vehicle, RigidBody& body, GearboxMode gearbox)
{
    return buildTruck(vehicle, body, gearbox, false);
}

GroundVehicleLayout buildTruck8x4(GroundVehicle& vehicle, RigidBody& body, GearboxMode gearbox)
{
    return buildTruck(vehicle, body, gearbox, true);
}

GroundVehicleLayout buildSemiTrailer(GroundVehicle& vehicle, RigidBody& body, f32 payload, f32 couplingHeight)
{
    constexpr f32 kEmpty = 7000.0f;
    constexpr f32 kRestHeight = 1.8f;
    // Kingpin 4.6 m ahead of the centre of mass, tri-axle middle 7.7 m behind it: the kingpin
    // carries 3.1 / 7.7 of the weight.
    constexpr f32 kKingpinZ = 4.6f;
    constexpr f32 kAxleGroupZ = kKingpinZ - 7.7f;
    constexpr f32 kAxleSpacing = 1.31f;

    payload = isfinite(payload) ? Max(payload, 0.0f) : 12000.0f;
    couplingHeight = isfinite(couplingHeight) ? Clamp(couplingHeight, 0.5f, 2.0f) : 1.2f;
    const f32 mass = kEmpty + payload;
    GroundVehicleLayout layout;
    layout.mass = mass;
    layout.restHeight = kRestHeight;
    layout.wheelbase = kKingpinZ - kAxleGroupZ;
    // Floor and chassis only; the box body above is not a collider.
    layout.collisionHalfExtents = Math::vec3(1.25f, 0.5f, 6.8f);
    layout.couplingPoint = Math::vec3(0.0f, couplingHeight - kRestHeight, kKingpinZ);
    preset::setupBody(body, mass, Math::vec3(1.25f, 1.3f, 6.8f));

    const f32 weight = mass * preset::kGravity;
    const f32 groupLoad = weight * kKingpinZ / (kKingpinZ - kAxleGroupZ);
    const f32 axleLoad = groupLoad / 3.0f;
    for (s32 k = -1; k <= 1; ++k)
    {
        AxleParameters a;
        a.position = kAxleGroupZ - static_cast<f32>(k) * kAxleSpacing;
        a.halfTrack = 1.02f;
        a.wheel.radius = 0.53f;
        a.wheel.width = 0.385f;
        a.wheel.inertia = 18.0f;
        a.wheel.tyre = TyreParameters::truck(false);
        a.wheel.suspension = SuspensionParameters::forLoad(0.5f * 112000.0f / 3.0f, 1.5f, 0.3f, 0.2f, 0.6f);
        a.wheel.maxBrakeTorque = 9000.0f;
        a.wheel.maxHandbrakeTorque = 9000.0f;
        a.wheel.bearingFriction = 6.0f;
        a.antiRollStiffness = 120000.0f;
        a.height = preset::axleHeightFor(a, axleLoad, kRestHeight);
        vehicle.addAxle(a);
    }
    layout.track = 2.04f;
    vehicle.setAerodynamics(2.0f, Math::vec3(0.0f, 1.0f, 0.0f));
    return layout;
}

} // namespace kinetix::Physics
