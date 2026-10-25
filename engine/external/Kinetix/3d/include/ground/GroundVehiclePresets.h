#ifndef KINETIX_PHYSICS_GROUND_GROUNDVEHICLEPRESETS_H
#define KINETIX_PHYSICS_GROUND_GROUNDVEHICLEPRESETS_H

#include "ground/GroundVehicle.h"
#include "ground/Transmission.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class RigidBody;

// Body space (+Z forward, +Y up, right -X, origin at the centre of mass).
struct GroundVehicleLayout
{
    f32 mass = 0.0f;
    Math::vec3 collisionHalfExtents{0.5f};
    // Height of the centre of mass above flat ground at static equilibrium, level.
    f32 restHeight = 0.0f;
    f32 wheelbase = 0.0f;
    f32 track = 0.0f;
    Math::vec3 towPoint{0.0f};
    Math::vec3 couplingPoint{0.0f};
    s32 frontDifferential = -1;
    s32 rearDifferential = -1;
    s32 centreDifferential = -1;
    s32 secondRearDifferential = -1;
};

enum class DriveLayout : u8
{
    FrontWheel,
    RearWheel,
    AllWheel
};

struct PassengerCarOptions
{
    DriveLayout drive = DriveLayout::FrontWheel;
    GearboxMode gearbox = GearboxMode::Manual;
    bool autoClutch = true;
    bool antiLock = false;
};

// 1400 kg five-door hatchback, 2.6 m wheelbase, 100 kW petrol, five-speed box, open
// differentials (all-wheel: open 40/60 centre differential).
GroundVehicleLayout buildPassengerCar(GroundVehicle& vehicle, RigidBody& body,
                                      const PassengerCarOptions& options = PassengerCarOptions());

// 9 t 6x4 tractor unit, 300 kW diesel through a 12-speed box, open inter-axle differential
// and two open axle differentials, all lockable; fifth wheel at about 1.2 m.
GroundVehicleLayout buildTruck6x4(GroundVehicle& vehicle, RigidBody& body,
                                  GearboxMode gearbox = GearboxMode::Automatic);
// 8x4 rigid variant: second steered front axle 1.9 m behind the first, 12 t.
GroundVehicleLayout buildTruck8x4(GroundVehicle& vehicle, RigidBody& body,
                                  GearboxMode gearbox = GearboxMode::Automatic);

// 13.6 m tri-axle semi-trailer, 7 t empty plus `payload` (kg), kingpin carrying about 40%,
// at `couplingHeight` above the ground at rest. Unpowered, braked on every wheel.
GroundVehicleLayout buildSemiTrailer(GroundVehicle& vehicle, RigidBody& body, f32 payload = 12000.0f,
                                     f32 couplingHeight = 1.2f);

// 5 t, 100 kW four-wheel-drive farm tractor, 40/60 weight split, no springs (the tyres are
// the suspension), front beam axle oscillating +-10 degrees, ten-speed box, lockable rear
// differential and limited-slip front one. Manual box with the clutch automated.
GroundVehicleLayout buildFarmTractor(GroundVehicle& vehicle, RigidBody& body);

// Tandem-axle drawbar farm trailer, 2.5 t empty plus `payload`, brakes following the
// tractor's; drawbar eye at `hitchHeight` at rest, carrying about 8% of the weight.
GroundVehicleLayout buildFarmTrailer(GroundVehicle& vehicle, RigidBody& body, f32 payload = 5000.0f,
                                     f32 hitchHeight = 0.5f);

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_GROUNDVEHICLEPRESETS_H
