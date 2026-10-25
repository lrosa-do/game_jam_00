#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/GroundVehiclePresets.h"

#include "PresetSupport.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

GroundVehicleLayout buildFarmTractor(GroundVehicle& vehicle, RigidBody& body)
{
    constexpr f32 kMass = 5000.0f;
    constexpr f32 kWheelbase = 2.6f;
    constexpr f32 kFrontShare = 0.4f;
    constexpr f32 kRestHeight = 1.0f;
    constexpr f32 kRearRadius = 0.9f;
    constexpr f32 kFrontRadius = 0.7f;

    GroundVehicleLayout layout;
    layout.mass = kMass;
    layout.wheelbase = kWheelbase;
    layout.restHeight = kRestHeight;
    layout.track = 1.9f;
    layout.collisionHalfExtents = Math::vec3(0.75f, 0.5f, 1.8f);
    preset::setupBody(body, kMass, Math::vec3(0.9f, 0.8f, 1.8f));

    const f32 frontZ = (1.0f - kFrontShare) * kWheelbase;
    const f32 rearZ = frontZ - kWheelbase;
    layout.towPoint = Math::vec3(0.0f, 0.5f - kRestHeight, rearZ - 1.0f);
    const f32 weight = kMass * preset::kGravity;
    const f32 frontLoad = kFrontShare * weight;
    const f32 rearLoad = weight - frontLoad;

    AxleParameters rear;
    rear.position = rearZ;
    rear.halfTrack = 0.95f;
    rear.wheel.radius = kRearRadius;
    rear.wheel.width = 0.52f;
    rear.wheel.inertia = 80.0f;
    rear.wheel.tyre = TyreParameters::tractorLug(15000.0f);
    rear.wheel.suspension.travel = 0.1f;
    rear.wheel.suspension.stiffness = 400000.0f;
    rear.wheel.suspension.preload = 0.0f;
    rear.wheel.suspension.bumpDamping = 7000.0f;
    rear.wheel.suspension.reboundDamping = 7000.0f;
    rear.wheel.suspension.bumpStopRange = 0.02f;
    rear.wheel.suspension.bumpStopStiffness = 2000000.0f;
    rear.wheel.maxBrakeTorque = 9000.0f;
    rear.wheel.maxHandbrakeTorque = 9000.0f;
    rear.wheel.bearingFriction = 15.0f;
    rear.height = preset::axleHeightFor(rear, rearLoad, kRestHeight);

    AxleParameters front;
    front.position = frontZ;
    front.halfTrack = 0.93f;
    front.steered = true;
    front.oscillating = true;
    front.oscillationLimit = 10.0f * 3.14159265f / 180.0f;
    front.wheel.radius = kFrontRadius;
    front.wheel.width = 0.42f;
    front.wheel.inertia = 18.0f;
    front.wheel.tyre = TyreParameters::tractorLug(10000.0f);
    front.wheel.suspension.travel = 0.12f;
    front.wheel.suspension.stiffness = 300000.0f;
    front.wheel.suspension.preload = 0.0f;
    front.wheel.suspension.bumpDamping = 5000.0f;
    front.wheel.suspension.reboundDamping = 5000.0f;
    front.wheel.suspension.bumpStopRange = 0.02f;
    front.wheel.suspension.bumpStopStiffness = 1500000.0f;
    front.wheel.maxBrakeTorque = 1500.0f;
    front.wheel.bearingFriction = 8.0f;
    front.height = preset::axleHeightFor(front, frontLoad, kRestHeight);

    const u32 rearAxle = vehicle.addAxle(rear);
    const u32 frontAxle = vehicle.addAxle(front);

    SteeringParameters steering;
    steering.maxAngle = 0.65f;
    steering.rate = 1.0f;
    steering.ackermann = 1.0f;
    vehicle.setSteering(steering);

    EngineParameters engine =
        EngineParameters::diesel(100000.0f, 2200.0f, 560.0f, 1400.0f, 1600.0f, 800.0f, 2400.0f);
    engine.inertia = 1.5f;
    engine.frictionTorque = 25.0f;
    engine.frictionPerRpm = 0.025f;
    engine.idleGain = 3.0f;
    engine.starterTorque = 150.0f;
    vehicle.engine().setParameters(engine);
    vehicle.engine().start();

    ClutchParameters clutch;
    clutch.maxTorque = 1200.0f;
    vehicle.clutch().setParameters(clutch);

    GearboxParameters gearbox;
    const f32 ratios[10] = {12.0f, 9.0f, 6.8f, 5.1f, 3.8f, 2.9f, 2.2f, 1.65f, 1.25f, 0.95f};
    gearbox.setRatios(ratios, 10);
    gearbox.reverseRatio = 10.5f;
    gearbox.finalDrive = 25.0f;
    gearbox.efficiency = 0.92f;
    gearbox.dragTorque = 4.0f;
    gearbox.inputInertia = 0.1f;
    gearbox.shiftTime = 0.4f;
    gearbox.upshiftRpmLight = 1500.0f;
    gearbox.upshiftRpmFull = 2250.0f;
    gearbox.downshiftRpmLight = 1000.0f;
    gearbox.downshiftRpmFull = 1500.0f;
    vehicle.gearbox().setParameters(gearbox);
    vehicle.gearbox().setMode(GearboxMode::Manual);
    vehicle.setAutoClutch(true);
    vehicle.setAutoClutchSpeeds(900.0f, 1200.0f, 1600.0f, 0.4f);

    // Rear: open, lockable. Front: lockable clutch-type limited slip, geared up by the tyre
    // radius ratio so both axles' tyres cover the same ground per turn of the front-drive coupling.
    DifferentialParameters rearDiff;
    rearDiff.type = DifferentialType::Open;
    rearDiff.lockable = true;
    rearDiff.inputInertia = 0.5f;
    layout.rearDifferential = static_cast<s32>(
        vehicle.addDifferential(rearDiff, DriveOutput::wheel(2 * rearAxle), DriveOutput::wheel(2 * rearAxle + 1)));
    DifferentialParameters frontDiff;
    frontDiff.type = DifferentialType::ClutchLimitedSlip;
    frontDiff.preloadTorque = 300.0f;
    frontDiff.torqueSensing = 0.2f;
    frontDiff.lockable = true;
    frontDiff.ratio = kFrontRadius / kRearRadius;
    frontDiff.inputInertia = 0.2f;
    layout.frontDifferential = static_cast<s32>(vehicle.addDifferential(
        frontDiff, DriveOutput::wheel(2 * frontAxle), DriveOutput::wheel(2 * frontAxle + 1)));
    DifferentialParameters coupling;
    coupling.type = DifferentialType::Locked;
    coupling.split = kFrontShare;
    coupling.inputInertia = 0.2f;
    layout.centreDifferential = static_cast<s32>(
        vehicle.addDifferential(coupling, DriveOutput::differential(static_cast<u32>(layout.frontDifferential)),
                                DriveOutput::differential(static_cast<u32>(layout.rearDifferential))));
    vehicle.setDriveRoot(layout.centreDifferential);
    vehicle.setAerodynamics(3.0f, Math::vec3(0.0f, 0.5f, 0.0f));
    return layout;
}

GroundVehicleLayout buildFarmTrailer(GroundVehicle& vehicle, RigidBody& body, f32 payload, f32 hitchHeight)
{
    constexpr f32 kEmpty = 2500.0f;
    constexpr f32 kRestHeight = 1.2f;
    constexpr f32 kDrawbarZ = 3.5f;
    constexpr f32 kTandemZ = -0.3f;
    constexpr f32 kTandemSpread = 1.0f;

    payload = isfinite(payload) ? Max(payload, 0.0f) : 5000.0f;
    hitchHeight = isfinite(hitchHeight) ? Clamp(hitchHeight, 0.2f, 1.5f) : 0.5f;
    const f32 mass = kEmpty + payload;
    GroundVehicleLayout layout;
    layout.mass = mass;
    layout.restHeight = kRestHeight;
    layout.wheelbase = kDrawbarZ - kTandemZ;
    layout.track = 2.0f;
    layout.collisionHalfExtents = Math::vec3(1.2f, 0.45f, 2.6f);
    layout.couplingPoint = Math::vec3(0.0f, hitchHeight - kRestHeight, kDrawbarZ);
    preset::setupBody(body, mass, Math::vec3(1.2f, 0.8f, 2.6f));

    const f32 weight = mass * preset::kGravity;
    const f32 groupLoad = weight * kDrawbarZ / (kDrawbarZ - kTandemZ);
    for (s32 k = 0; k < 2; ++k)
    {
        AxleParameters a;
        a.position = kTandemZ + (k == 0 ? 0.5f : -0.5f) * kTandemSpread;
        a.halfTrack = 1.0f;
        a.wheel.radius = 0.5f;
        a.wheel.width = 0.4f;
        a.wheel.inertia = 10.0f;
        a.wheel.tyre = TyreParameters::tractorLug(12000.0f);
        a.wheel.suspension = SuspensionParameters::forLoad(0.25f * groupLoad, 2.0f, 0.3f, 0.08f, 0.5f);
        a.wheel.maxBrakeTorque = 4000.0f;
        a.wheel.maxHandbrakeTorque = 4000.0f;
        a.wheel.bearingFriction = 6.0f;
        a.height = preset::axleHeightFor(a, 0.5f * groupLoad, kRestHeight);
        vehicle.addAxle(a);
    }
    vehicle.setAerodynamics(2.0f);
    return layout;
}

} // namespace kinetix::Physics
