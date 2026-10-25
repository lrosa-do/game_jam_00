#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/GroundVehiclePresets.h"

#include "PresetSupport.h"
#include "dynamics/RigidBody.h"

namespace kinetix::Physics
{

GroundVehicleLayout buildPassengerCar(GroundVehicle& vehicle, RigidBody& body, const PassengerCarOptions& options)
{
    constexpr f32 kMass = 1400.0f;
    constexpr f32 kWheelbase = 2.6f;
    constexpr f32 kFrontShare = 0.6f;
    constexpr f32 kRestHeight = 0.55f;
    constexpr f32 kHalfTrack = 0.75f;
    constexpr f32 kRadius = 0.316f;

    GroundVehicleLayout layout;
    layout.mass = kMass;
    layout.wheelbase = kWheelbase;
    layout.track = 2.0f * kHalfTrack;
    layout.restHeight = kRestHeight;
    layout.collisionHalfExtents = Math::vec3(0.85f, 0.4f, 2.15f);
    layout.towPoint = Math::vec3(0.0f, 0.45f - kRestHeight, -2.25f);

    preset::setupBody(body, kMass, Math::vec3(0.8f, 0.5f, 2.0f));

    const f32 frontZ = (1.0f - kFrontShare) * kWheelbase;
    const f32 rearZ = frontZ - kWheelbase;
    const f32 weight = kMass * preset::kGravity;
    const f32 frontLoad = kFrontShare * weight;
    const f32 rearLoad = weight - frontLoad;

    WheelParameters wheel;
    wheel.radius = kRadius;
    wheel.width = 0.205f;
    wheel.tyre = TyreParameters::carRoad();
    wheel.bearingFriction = 0.5f;

    const bool frontDriven = options.drive != DriveLayout::RearWheel;
    const bool rearDriven = options.drive != DriveLayout::FrontWheel;

    AxleParameters front;
    front.position = frontZ;
    front.halfTrack = kHalfTrack;
    front.steered = true;
    front.wheel = wheel;
    front.wheel.inertia = frontDriven ? 1.1f : 0.9f;
    front.wheel.suspension = SuspensionParameters::forLoad(0.5f * frontLoad, 1.3f, 0.35f, 0.2f, 0.6f);
    front.wheel.maxBrakeTorque = 1800.0f;
    front.wheel.camber = -0.01f;
    front.antiRollStiffness = 20000.0f;
    front.height = preset::axleHeightFor(front, frontLoad, kRestHeight);

    AxleParameters rear;
    rear.position = rearZ;
    rear.halfTrack = kHalfTrack;
    rear.wheel = wheel;
    rear.wheel.inertia = rearDriven ? 1.1f : 0.9f;
    rear.wheel.suspension = SuspensionParameters::forLoad(0.5f * rearLoad, 1.5f, 0.35f, 0.2f, 0.6f);
    rear.wheel.maxBrakeTorque = 900.0f;
    rear.wheel.maxHandbrakeTorque = 1500.0f;
    rear.wheel.camber = -0.01f;
    rear.antiRollStiffness = 6000.0f;
    rear.height = preset::axleHeightFor(rear, rearLoad, kRestHeight);

    const u32 frontAxle = vehicle.addAxle(front);
    const u32 rearAxle = vehicle.addAxle(rear);

    SteeringParameters steering;
    steering.maxAngle = 0.6f;
    steering.rate = 1.5f;
    steering.ackermann = 1.0f;
    vehicle.setSteering(steering);

    EngineParameters engine = EngineParameters::gasoline(100000.0f, 6000.0f, 180.0f, 4000.0f, 800.0f, 6500.0f);
    engine.inertia = 0.13f;
    // Closed-throttle friction: ~50 N m of engine braking at 6000 rpm, ~20 N m near idle.
    engine.frictionTorque = 15.0f;
    engine.frictionPerRpm = 0.006f;
    engine.starterTorque = 40.0f;
    vehicle.engine().setParameters(engine);
    vehicle.engine().start();

    ClutchParameters clutch;
    clutch.maxTorque = 320.0f;
    vehicle.clutch().setParameters(clutch);

    GearboxParameters gearbox;
    const f32 ratios[5] = {3.6f, 2.1f, 1.4f, 1.03f, 0.8f};
    gearbox.setRatios(ratios, 5);
    gearbox.reverseRatio = 3.5f;
    gearbox.finalDrive = 4.1f;
    gearbox.efficiency = 0.96f;
    gearbox.dragTorque = 0.5f;
    gearbox.inputInertia = 0.02f;
    gearbox.shiftTime = 0.25f;
    gearbox.upshiftRpmLight = 2500.0f;
    gearbox.upshiftRpmFull = 6200.0f;
    gearbox.downshiftRpmLight = 1200.0f;
    gearbox.downshiftRpmFull = 3200.0f;
    gearbox.minShiftInterval = 1.0f;
    vehicle.gearbox().setParameters(gearbox);
    vehicle.gearbox().setMode(options.gearbox);
    vehicle.setAutoClutch(options.autoClutch);
    vehicle.setAntiLockBrakes(options.antiLock);

    DifferentialParameters axleDiff;
    axleDiff.type = DifferentialType::Open;
    axleDiff.inputInertia = 0.04f;
    if (options.drive == DriveLayout::AllWheel)
    {
        layout.frontDifferential = static_cast<s32>(vehicle.addDifferential(
            axleDiff, DriveOutput::wheel(2 * frontAxle), DriveOutput::wheel(2 * frontAxle + 1)));
        layout.rearDifferential = static_cast<s32>(vehicle.addDifferential(
            axleDiff, DriveOutput::wheel(2 * rearAxle), DriveOutput::wheel(2 * rearAxle + 1)));
        DifferentialParameters centre;
        centre.type = DifferentialType::Open;
        centre.split = 0.4f;
        centre.lockable = true;
        centre.inputInertia = 0.05f;
        layout.centreDifferential = static_cast<s32>(
            vehicle.addDifferential(centre, DriveOutput::differential(static_cast<u32>(layout.frontDifferential)),
                                    DriveOutput::differential(static_cast<u32>(layout.rearDifferential))));
        vehicle.setDriveRoot(layout.centreDifferential);
    }
    else if (frontDriven)
    {
        layout.frontDifferential = static_cast<s32>(vehicle.addDifferential(
            axleDiff, DriveOutput::wheel(2 * frontAxle), DriveOutput::wheel(2 * frontAxle + 1)));
        vehicle.setDriveRoot(layout.frontDifferential);
    }
    else
    {
        layout.rearDifferential = static_cast<s32>(vehicle.addDifferential(
            axleDiff, DriveOutput::wheel(2 * rearAxle), DriveOutput::wheel(2 * rearAxle + 1)));
        vehicle.setDriveRoot(layout.rearDifferential);
    }

    vehicle.setAerodynamics(0.66f, Math::vec3(0.0f, 0.1f, 0.2f));
    return layout;
}

} // namespace kinetix::Physics
