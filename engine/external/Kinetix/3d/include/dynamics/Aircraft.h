#ifndef KINETIX_PHYSICS_DYNAMICS_AIRCRAFT_H
#define KINETIX_PHYSICS_DYNAMICS_AIRCRAFT_H

#include "dynamics/Atmosphere.h"
#include "dynamics/DragElement.h"
#include "dynamics/PistonEngine.h"
#include "dynamics/Propeller.h"
#include "dynamics/Wing.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RaycastVehicle;
class RigidBody;

// The pilot's inputs. Sign conventions are the cockpit's: aileron +1 rolls
// right, elevator +1 (stick back) pitches the nose up, rudder +1 yaws the
// nose right. Flaps and throttle run 0..1. The trims are added to the
// matching input before mixing - trim() writes them, so a trimmed aircraft
// flies hands-off with every input at zero.
struct ControlInputs
{
    f32 aileron = 0.0f;
    f32 elevator = 0.0f;
    f32 rudder = 0.0f;
    f32 flaps = 0.0f;
    f32 throttle = 0.0f;
    f32 aileronTrim = 0.0f;
    f32 elevatorTrim = 0.0f;
    f32 rudderTrim = 0.0f;
};

// What the aircraft is doing, measured on the last applyForces() or
// computeForces(). Angles in radians, speeds in m/s.
struct AircraftTelemetry
{
    f32 trueAirspeed = 0.0f;
    // TAS scaled by sqrt(rho / rho0): what an airspeed indicator shows.
    f32 equivalentAirspeed = 0.0f;
    f32 mach = 0.0f;
    // Between the flight path through the air and the body's forward axis,
    // in the body's symmetry plane; positive nose above the flight path.
    f32 angleOfAttack = 0.0f;
    // Positive when the relative wind comes from the right.
    f32 sideslip = 0.0f;
    f32 altitude = 0.0f;
    f32 verticalSpeed = 0.0f;
    // Attitude: nose above the horizon, right wing down, and heading
    // turning right from world +Z.
    f32 pitch = 0.0f;
    f32 bank = 0.0f;
    f32 heading = 0.0f;
    // Roll, pitch and yaw rates (p, q, r) about the forward, right and down
    // axes: positive is rolling right, nose rising, nose swinging right.
    Math::vec3 bodyRates{0.0f};
    // Total aerodynamic and propulsive force (world) and torque about the
    // centre of mass (world).
    Math::vec3 force{0.0f};
    Math::vec3 torque{0.0f};
    f32 density = Atmosphere::SeaLevelDensity;
};

// What trim() should hold. Exactly one of climbAngle and throttle is solved
// for: with throttle negative the throttle is found for the given climb
// angle (level flight by default), otherwise the throttle is held and the
// climb angle found - a glide is throttle = 0.
struct TrimTarget
{
    f32 airspeed = 50.0f; // true airspeed
    f32 climbAngle = 0.0f;
    f32 throttle = -1.0f;
    f32 flaps = 0.0f;
    Math::vec3 gravity{0.0f, -9.81f, 0.0f};
};

struct TrimResult
{
    bool converged = false;
    u32 iterations = 0;
    // Largest remaining force (in weights) or moment (in weight x metre).
    f32 residual = 0.0f;
    f32 angleOfAttack = 0.0f;
    f32 sideslip = 0.0f;
    f32 climbAngle = 0.0f;
    f32 throttle = 0.0f;
    f32 elevator = 0.0f;
    f32 aileron = 0.0f;
    f32 rudder = 0.0f;
    f32 rpm = 0.0f;
};

// A rigid body with wings, propellers and engines, and the controls that move
// them. It does not own the body, add a shape or provide landing gear
// (see addLightAircraftGear).
//
// Call applyForces(step) once per fixed step before the world integrates:
//  - driving PhysicsWorld::step() yourself: just before each step();
//  - using PhysicsWorld::update(): register stepCallback with setStepCallback()
//    and call applyForces() once by hand before the first update. Calling it
//    once per update() would feed only the first fixed step, since integration
//    clears the force accumulators.
//
// Body axes default to +Z forward, +Y up (right wing on -X); see setAxes().
// Wings, propellers and drag elements are all given in this body space.
class Aircraft
{
public:
    Aircraft();

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    RigidBody* body() const
    {
        return mBody;
    }

    // A shared atmosphere, so several aircraft fly through the same wind. Null
    // goes back to this aircraft's own one.
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

    // Body-space forward and up; made orthonormal.
    void setAxes(const Math::vec3& forward, const Math::vec3& up);
    const Math::vec3& forwardAxis() const
    {
        return mForward;
    }
    const Math::vec3& upAxis() const
    {
        return mUp;
    }
    Math::vec3 rightAxis() const
    {
        return Math::cross(mForward, mUp);
    }

    // Each returns the index of what it added. Wings and propellers are held
    // by value, so a reference from wing() or propeller() lasts only until
    // the next add.
    u32 addWing(const Wing& wing);
    u32 addEngine(const PistonEngine& engine);
    // `engine` is an index from addEngine(), or -1 for a propeller driven by
    // its own rpm or shaft torque setting.
    u32 addPropeller(const Propeller& propeller, s32 engine = -1);
    u32 addDragElement(const DragElement& element);

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
    u32 engineCount() const
    {
        return static_cast<u32>(mEngines.size());
    }
    PistonEngine& engine(u32 index)
    {
        return mEngines[index];
    }
    const PistonEngine& engine(u32 index) const
    {
        return mEngines[index];
    }
    u32 propellerCount() const
    {
        return static_cast<u32>(mPropellers.size());
    }
    Propeller& propeller(u32 index)
    {
        return mPropellers[index].propeller;
    }
    const Propeller& propeller(u32 index) const
    {
        return mPropellers[index].propeller;
    }
    s32 propellerEngine(u32 index) const
    {
        return mPropellers[index].engine;
    }
// The wake of `sourceWing` reaches `wing`: its strips see the air pushed down
// by factor * alpha_i * V (alpha_i the source's induced angle, V the true
// airspeed). Far-wake downwash is 2 alpha_i at the wing, ~1.5-2x at a tail a
// few chords behind. A source added after the target is read from its previous
// evaluation. -1 removes the link.
    void setDownwash(u32 wing, s32 sourceWing, f32 factor = 1.7f);

    u32 dragElementCount() const
    {
        return static_cast<u32>(mDragElements.size());
    }
    DragElement& dragElement(u32 index)
    {
        return mDragElements[index];
    }

    // Inputs are clamped to their ranges.
    void setControls(const ControlInputs& controls);
    const ControlInputs& controls() const
    {
        return mControls;
    }

    // Evaluates every surface, propeller and drag element at the body's
    // current state and adds the result to the body. Advances the propeller
    // speeds by `step`.
    void applyForces(f32 step);
    // The same evaluation, nothing applied and no propeller speed changed;
    // world space, torque about the centre of mass. Updates the telemetry.
    void computeForces(Math::vec3& force, Math::vec3& torque);

    // For PhysicsWorld::setStepCallback, with the Aircraft as userData.
    static void stepCallback(f32 step, void* userData);

    const AircraftTelemetry& telemetry() const
    {
        return mTelemetry;
    }

// Newton's method on the six force and moment balances for steady, straight,
// wings-level flight at the target's airspeed. On success the body is put in
// that state (position and heading kept, velocity along the flight path, no
// rotation), propellers at balanced speed, trims and throttle written into the
// controls. On failure nothing is changed.
    TrimResult trim(const TrimTarget& target);

private:
    struct PropellerEntry
    {
        Propeller propeller;
        s32 engine = -1;
    };

    void updateControlSurfaces();
    void driveEngines(f32 density);
    void accumulate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque);
    void updateTelemetry(const Math::vec3& force, const Math::vec3& torque);
    f32 balancedPropellerSpeed(u32 index, f32 inflow, f32 density) const;

    RigidBody* mBody = nullptr;
    Atmosphere mAtmosphere;
    const Atmosphere* mSharedAtmosphere = nullptr;
    Math::vec3 mForward{0.0f, 0.0f, 1.0f};
    Math::vec3 mUp{0.0f, 1.0f, 0.0f};

    struct DownwashLink
    {
        s32 source = -1;
        f32 factor = 0.0f;
    };

    ct::Vector<Wing> mWings;
    ct::Vector<DownwashLink> mDownwash;
    ct::Vector<PistonEngine> mEngines;
    ct::Vector<PropellerEntry> mPropellers;
    ct::Vector<DragElement> mDragElements;

    ControlInputs mControls;
    AircraftTelemetry mTelemetry;
};

// Where the default light aircraft's parts are, for whoever adds its
// collision shape and landing gear. Body space, +Z forward, +Y up.
struct LightAircraftLayout
{
    // Suspension hard points; the struts point straight down.
    Math::vec3 noseGear{0.0f};
    Math::vec3 leftMainGear{0.0f};
    Math::vec3 rightMainGear{0.0f};
    f32 gearRestLength = 0.5f;
    f32 wheelRadius = 0.25f;
    Math::vec3 fuselageHalfExtents{0.5f};
    // Height of the body origin above flat ground with the gear at rest -
    // where to put the body to start it standing on its wheels.
    f32 restHeight = 1.0f;
    u32 mainWing = 0;
    u32 horizontalTail = 1;
    u32 verticalTail = 2;
};

// A generic high-wing, single-engine, four-seat light aircraft: 1000 kg,
// 11 m span, 16 m^2 wing, 120 kW engine. Sets the body's mass, inertia,
// damping and sleep, and fills the aircraft with wings, engine, propeller and
// fuselage drag. Stalls clean at about 27 m/s.
LightAircraftLayout buildLightAircraft(Aircraft& aircraft, RigidBody& body);

// Tricycle undercarriage for the aircraft above on a RaycastVehicle built over
// the same body: wheel 0 is the nose wheel (steer it with the rudder), 1 and
// 2 the mains. The wheels roll freely; the vehicle only provides suspension
// and sideways grip.
void addLightAircraftGear(RaycastVehicle& gear, const LightAircraftLayout& layout);

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_AIRCRAFT_H
