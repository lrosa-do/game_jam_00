#ifndef KINETIX_PHYSICS_ROTORCRAFT_HELICOPTER_H
#define KINETIX_PHYSICS_ROTORCRAFT_HELICOPTER_H

#include "dynamics/Atmosphere.h"
#include "dynamics/DragElement.h"
#include "dynamics/Wing.h"
#include "rotorcraft/Rotor.h"
#include "rotorcraft/RotorTransmission.h"
#include "rotorcraft/TurboshaftEngine.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class PhysicsWorld;
class RigidBody;

// The pilot's inputs, cockpit sign conventions. Collective 0..1 (lever down to fully up). Cyclic
// -1..1: longitudinal +1 is stick forward (disc and nose down), lateral +1 stick right. Pedals
// -1..1, +1 right pedal (nose right). Throttle is the twist grip, 0 idle to 1 flight (governed).
struct HelicopterControls
{
    f32 collective = 0.0f;
    f32 cyclicLongitudinal = 0.0f;
    f32 cyclicLateral = 0.0f;
    f32 pedals = 0.0f;
    f32 throttle = 1.0f;
    bool engineOn = true;
};

// How the controls move the blades: swashplate collective range (pitch at 0.75 R, rad, at lever
// 0 and 1), cyclic per unit stick, tail rotor collective at centred pedals and per unit pedal
// (the sign is the rigging's: with the default tail rotor right pedal reduces its pitch).
struct HelicopterRigging
{
    f32 collectiveMin = 0.0f;
    f32 collectiveMax = 0.25f;
    f32 longitudinalCyclic = 0.17f;
    f32 lateralCyclic = 0.12f;
    f32 tailCollective = 0.12f;
    f32 tailCollectivePerPedal = -0.22f;
};

// Rate damping added on top of the stick: cyclic and pedal per rad/s of body rate, clamped to
// `authority` of full travel (a limited-authority series actuator).
struct StabilityAugmentation
{
    bool enabled = false;
    f32 pitch = 0.4f;
    f32 roll = 0.25f;
    f32 yaw = 0.6f;
    f32 authority = 0.25f;
};

// A landing-gear contact point (skid end, wheel, float): spring-damper against the ground with
// Coulomb friction, held by an anchored spring (stiction) so a helicopter on its skids with the
// rotor turning does not creep.
struct SkidContact
{
    Math::vec3 position{0.0f}; // body space
    f32 stiffness = 70000.0f;  // N/m
    f32 damping = 5000.0f;     // N s/m
    f32 tangentialStiffness = 30000.0f;
    f32 tangentialDamping = 2500.0f;
    f32 friction = 0.6f;
};

struct SkidContactState
{
    bool inContact = false;
    f32 normalForce = 0.0f;
    f32 penetration = 0.0f;
    bool sliding = false;
    Math::vec3 point{0.0f}; // world
    Math::vec3 anchor{0.0f};
    bool anchored = false;
};

struct GroundHit
{
    f32 distance = 0.0f;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Measured on the last applyForces() or computeForces(). Angles in rad, speeds in m/s, power in W.
struct HelicopterTelemetry
{
    f32 rotorRpm = 0.0f;
    f32 rotorSpeedPercent = 0.0f;
    f32 tailRotorRpm = 0.0f;
    f32 turbineSpeedPercent = 0.0f; // Np
    f32 gasGeneratorPercent = 0.0f; // Ng
    f32 mainRotorTorque = 0.0f;     // aerodynamic, N m
    f32 tailRotorTorque = 0.0f;
    // Shaft power the two rotors absorb, and what the engine delivers through the freewheel unit.
    f32 powerRequired = 0.0f;
    f32 enginePower = 0.0f;
    f32 fuelFlow = 0.0f; // kg/s
    f32 thrust = 0.0f;
    f32 tailRotorThrust = 0.0f;
    f32 inducedVelocity = 0.0f;
    f32 inflowRatio = 0.0f;
    f32 advanceRatio = 0.0f;
    f32 coning = 0.0f;
    f32 longitudinalFlapping = 0.0f; // a1s
    f32 lateralFlapping = 0.0f;      // b1s
    f32 discTiltForward = 0.0f;
    f32 discTiltRight = 0.0f;
    bool vortexRing = false;
    f32 groundEffectFactor = 1.0f;
    bool freewheelEngaged = true;
    // Of the lowest landing-gear point above the ground below the body; 0 standing on it, 1000
    // when no ground is found within 1000 m.
    f32 heightAboveGround = 0.0f;
    f32 altitude = 0.0f;
    f32 trueAirspeed = 0.0f;
    f32 groundSpeed = 0.0f;
    f32 verticalSpeed = 0.0f;
    Math::vec3 airVelocity{0.0f};
    // As for Aircraft: positive with the relative wind from the right, and nose above the flight path.
    f32 sideslip = 0.0f;
    f32 angleOfAttack = 0.0f;
    // Nose above the horizon, right side down, heading turning right from world +Z.
    f32 pitch = 0.0f;
    f32 bank = 0.0f;
    f32 heading = 0.0f;
    // Roll, pitch, yaw rates (p, q, r): rolling right, nose rising, nose swinging right.
    Math::vec3 bodyRates{0.0f};
    bool onGround = false;
    f32 gearLoad = 0.0f; // total normal force on the contacts, N
    // Everything the helicopter added to the body (world); torque about the centre of mass.
    Math::vec3 force{0.0f};
    Math::vec3 torque{0.0f};
    f32 density = Atmosphere::SeaLevelDensity;
};

// What trim() should hold: steady straight flight at a true airspeed along the current heading,
// climbing at climbRate (m/s), no sideslip, rotor at the governed speed.
struct HelicopterTrimTarget
{
    f32 airspeed = 0.0f;
    f32 climbRate = 0.0f;
    Math::vec3 gravity{0.0f, -9.81f, 0.0f};
};

struct HelicopterTrimResult
{
    bool converged = false;
    u32 iterations = 0;
    // Largest remaining force (in weights) or moment (in weight x metre).
    f32 residual = 0.0f;
    f32 collective = 0.0f;
    f32 cyclicLongitudinal = 0.0f;
    f32 cyclicLateral = 0.0f;
    f32 pedals = 0.0f;
    f32 pitch = 0.0f;
    f32 bank = 0.0f;
    f32 powerRequired = 0.0f; // shaft power, both rotors and the drive's friction
    f32 mainRotorPower = 0.0f;
    f32 tailRotorPower = 0.0f;
    f32 thrust = 0.0f;
    f32 longitudinalFlapping = 0.0f;
    f32 lateralFlapping = 0.0f;
};

// A rigid body with a main rotor, an anti-torque tail rotor, a turboshaft engine driving both
// through a freewheel unit and gearbox, stabilising surfaces, fuselage drag and skid landing
// gear. Like Aircraft it does not own the body; unlike Aircraft it brings its own landing gear
// and ground probe, since the rotor needs the ground distance (ground effect) anyway.
//
// The rotors act on the airframe as forces and moments at their hubs, their speeds an internal
// state of the drive train, not rigid bodies on hinges (a stiff gyroscopically coupled
// constraint at the heart of every step). The quasi-steady flapping solution carries what the
// hub passes on (thrust tilt, hub moment, damping); the shaft reaction comes from the drive
// train's angular momentum balance:
//     airframe torque = -(Q_aero + I_rotor domega/dt) about each shaft.
//
// applyForces order: controls and stability augmentation mixed onto swashplate and tail pitch;
// ground probed below the hub and each contact; both rotors advance at the body's current state;
// drive train integrates rotor and turbine speeds against the rotor and engine torques; engine
// fuel control and gas generator advance; then rotor loads, shaft reactions, surfaces and drag
// elements (in the rotor's wake where under it) and skid contacts are added to the body. Timing
// contract is Aircraft's: call applyForces(step) before each PhysicsWorld::step(), or register
// stepCallback and prime it once.
//
// Rotor wake on the airframe: a point under the disc, inside the wake column skewed back by the
// wake angle, sees the induced velocity grow from v at the disc to 2v a radius below
// (v (1 + z / sqrt(R^2 + z^2))), with a soft edge: the hover download on the fuselage and the
// tailplane's nose-up kick in transition.
class Helicopter
{
public:
    using GroundQuery = bool (*)(const Math::vec3& origin, const Math::vec3& direction,
                                 f32 maxDistance, GroundHit& hit, void* userData);

    Helicopter();

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    RigidBody* body() const
    {
        return mBody;
    }

    void setAtmosphere(const Atmosphere* atmosphere)
    {
        mSharedAtmosphere = atmosphere;
    }
    const Atmosphere& atmosphere() const
    {
        return mSharedAtmosphere ? *mSharedAtmosphere : mAtmosphere;
    }
    Atmosphere& ownAtmosphere()
    {
        return mAtmosphere;
    }

    // Ground: by default a horizontal plane at world y = groundHeight. A query replaces it (null
    // goes back to the plane); setGround() ray-casts a PhysicsWorld, ignoring the helicopter's own body.
    void setGroundPlane(f32 height)
    {
        mGroundHeight = height;
    }
    void setGroundQuery(GroundQuery query, void* userData)
    {
        mGroundQuery = query;
        mGroundUserData = userData;
        mGroundWorld = nullptr;
    }
    void setGround(const PhysicsWorld* world, u32 ignoredBody);
    // Probes the ground straight down (world -Y) from `origin`.
    bool probeGround(const Math::vec3& origin, f32 maxDistance, GroundHit& hit) const;

    Rotor& mainRotor()
    {
        return mMainRotor;
    }
    const Rotor& mainRotor() const
    {
        return mMainRotor;
    }
    Rotor& tailRotor()
    {
        return mTailRotor;
    }
    const Rotor& tailRotor() const
    {
        return mTailRotor;
    }
    // Off by default - describe the tail rotor first (buildLightHelicopter does both). Without
    // one the helicopter has no anti-torque device.
    void setTailRotorEnabled(bool enabled)
    {
        mHasTailRotor = enabled;
    }
    bool tailRotorEnabled() const
    {
        return mHasTailRotor;
    }
    TurboshaftEngine& engine()
    {
        return mEngine;
    }
    const TurboshaftEngine& engine() const
    {
        return mEngine;
    }
    // Its rotor inertias are the caller's to keep in step with the rotors (Rotor::polarInertia()).
    RotorTransmission& transmission()
    {
        return mTransmission;
    }
    const RotorTransmission& transmission() const
    {
        return mTransmission;
    }
    // Main rotor speed the governor holds, rad/s (100%); sets the engine ratio so the power
    // turbine is at its rated speed there.
    void setRatedRotorSpeed(f32 omega);
    f32 ratedRotorSpeed() const
    {
        return mRatedRotorSpeed;
    }

    u32 addWing(const Wing& wing);
    u32 wingCount() const
    {
        return static_cast<u32>(mWings.size());
    }
    Wing& wing(u32 index)
    {
        return mWings[index];
    }
    const Wing& wing(u32 index) const
    {
        return mWings[index];
    }
    u32 addDragElement(const DragElement& element);
    u32 dragElementCount() const
    {
        return static_cast<u32>(mDragElements.size());
    }
    DragElement& dragElement(u32 index)
    {
        return mDragElements[index];
    }
    void setRotorWakeOnAirframe(bool enabled)
    {
        mWakeOnAirframe = enabled;
    }

    u32 addSkidContact(const SkidContact& contact);
    u32 skidContactCount() const
    {
        return static_cast<u32>(mContacts.size());
    }
    const SkidContact& skidContact(u32 index) const
    {
        return mContacts[index].contact;
    }
    const SkidContactState& skidContactState(u32 index) const
    {
        return mContacts[index].state;
    }

    void setRigging(const HelicopterRigging& rigging)
    {
        mRigging = rigging;
    }
    const HelicopterRigging& rigging() const
    {
        return mRigging;
    }
    void setStabilityAugmentation(const StabilityAugmentation& sas)
    {
        mSas = sas;
    }
    const StabilityAugmentation& stabilityAugmentation() const
    {
        return mSas;
    }

    // Inputs are clamped to their ranges.
    void setControls(const HelicopterControls& controls);
    const HelicopterControls& controls() const
    {
        return mControls;
    }

    // Puts the drive at the rated rotor speed with the engine running and governor settled, or
    // everything stopped. Initial conditions.
    void startRunning();
    void shutDown();

    void applyForces(f32 step);
    // The same evaluation at the current rotor state: nothing applied, no state advanced. World
    // space, torque about the centre of mass.
    void computeForces(Math::vec3& force, Math::vec3& torque);
    static void stepCallback(f32 step, void* userData);

    const HelicopterTelemetry& telemetry() const
    {
        return mTelemetry;
    }

    // Finds collective, cyclic, pedals, pitch and bank for steady straight flight at the target
    // airspeed and climb rate on the current heading, by Levenberg-Marquardt on the six force and
    // moment balances, the rotors solved to steady state at the governed speed. On success the body
    // is left in that state (position and heading kept, no rotation), controls set, drive at the
    // governed speed, gas generator at the power needed. On failure nothing is changed.
    HelicopterTrimResult trim(const HelicopterTrimTarget& target);

private:
    enum class Evaluation : u8
    {
        Advance,
        Current,
        Steady
    };
    struct ContactEntry
    {
        SkidContact contact;
        SkidContactState state;
    };

    void mixControls();
    void accumulate(Evaluation mode, f32 step, Math::vec3& force, Math::vec3& torque);
    RotorOperatingPoint operatingPoint(const Rotor& rotor, f32 omega, f32 groundDistance) const;
    Math::vec3 wakeVelocity(const Math::vec3& bodyPoint) const;
    void contactForces(bool apply, Math::vec3& force, Math::vec3& torque);
    void updateTelemetry(const Math::vec3& force, const Math::vec3& torque);

    RigidBody* mBody = nullptr;
    Atmosphere mAtmosphere;
    const Atmosphere* mSharedAtmosphere = nullptr;
    f32 mGroundHeight = 0.0f;
    GroundQuery mGroundQuery = nullptr;
    void* mGroundUserData = nullptr;
    const PhysicsWorld* mGroundWorld = nullptr;
    u32 mGroundIgnoredBody = 0xFFFFFFFFu;

    Rotor mMainRotor;
    Rotor mTailRotor;
    bool mHasTailRotor = false;
    TurboshaftEngine mEngine;
    RotorTransmission mTransmission;
    f32 mRatedRotorSpeed = 40.0f;

    ct::Vector<Wing> mWings;
    ct::Vector<DragElement> mDragElements;
    ct::Vector<ContactEntry> mContacts;
    bool mWakeOnAirframe = true;

    HelicopterRigging mRigging;
    StabilityAugmentation mSas;
    HelicopterControls mControls;
    HelicopterTelemetry mTelemetry;
    f32 mHubGroundDistance = -1.0f;
};

// Part positions of the default light helicopter, for whoever adds its collision shape and
// draws it. Body space, +Z forward, +Y up.
struct LightHelicopterLayout
{
    // Skid ends: left front, left rear, right front, right rear.
    Math::vec3 skidPoints[4];
    // Box around the cabin, centred on the body origin and clear of the ground above the skids.
    Math::vec3 hullHalfExtents{0.5f};
    // Body origin height above flat ground resting on the skids.
    f32 restHeight = 1.0f;
    u32 horizontalStabiliser = 0;
    u32 fin = 1;
    // Hover trim at sea level (measured by the tests): pedal position, and the collective the
    // governor's anticipator is centred on.
    f32 hoverPedals = 0.0f;
    f32 hoverCollective = 0.0f;
};

// A generic five-seat light turbine helicopter: 1300 kg, three-blade articulated main rotor of
// 4.6 m radius (3.9% hinge offset, Lock number 5.5) turning anticlockwise seen from above at
// 415 rpm (200 m/s tip speed), two-blade teetering tail rotor on the left of the boom, 300 kW
// turboshaft, horizontal stabiliser, toed fin, skids. Sets the body's mass, inertia, damping and
// sleep and fills the helicopter (tail rotor enabled).
LightHelicopterLayout buildLightHelicopter(Helicopter& helicopter, RigidBody& body);

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ROTORCRAFT_HELICOPTER_H
