#ifndef KINETIX_PHYSICS_GROUND_GROUNDVEHICLE_H
#define KINETIX_PHYSICS_GROUND_GROUNDVEHICLE_H

#include "collision/CollisionFilter.h"
#include "ground/Engine.h"
#include "ground/Transmission.h"
#include "ground/Tyre.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class PhysicsWorld;
class RigidBody;
class VehicleSolver;

// Suspension along the strut (body -Y). Strut length L runs 0 at full bump to `travel` at
// full droop; compression x = travel - L. Spring force preload + stiffness * x (preload
// fades in over the first centimetre), progressive bump stop over the last bumpStopRange
// (rate reaches bumpStopStiffness at full bump), separate bump and rebound damping. Past
// full bump the contact is rigid. Without springs (tractor rear axle) the tyre's own
// vertical stiffness and damping act instead.
struct SuspensionParameters
{
    f32 travel = 0.2f;
    f32 stiffness = 30000.0f;  // N/m
    f32 preload = 0.0f;        // N
    f32 bumpDamping = 2500.0f; // N s/m, compressing
    f32 reboundDamping = 3500.0f;
    f32 bumpStopRange = 0.03f;
    f32 bumpStopStiffness = 300000.0f; // N/m at full bump

    // Spring for `staticLoad` (N on this wheel) at a ride frequency (Hz); damping ratio split
    // 0.7 / 1.3 between bump and rebound; sags to `sag` of the travel, stiffened to fit.
    static SuspensionParameters forLoad(f32 staticLoad, f32 frequency, f32 dampingRatio, f32 travel,
                                        f32 sag = 0.6f);
    f32 staticCompression(f32 load) const;
};

struct WheelParameters
{
    f32 radius = 0.31f;
    f32 width = 0.2f;
    // Wheel, tyre, hub, brake rotor and half-shaft about the axle, kg m^2.
    f32 inertia = 1.0f;
    TyreParameters tyre;
    SuspensionParameters suspension;
    f32 maxBrakeTorque = 1500.0f;    // N m at full pedal
    f32 maxHandbrakeTorque = 0.0f;   // N m at full lever
    f32 bearingFriction = 1.0f;      // N m
    // Static camber, rad; positive is the top of the wheel leaning outward.
    f32 camber = 0.0f;
};

// Left (+X) and right (-X) wheels; GroundVehicle numbers them 2 * axle (left), 2 * axle + 1.
struct AxleParameters
{
    f32 position = 0.0f;  // body Z of the axle line
    f32 height = 0.0f;    // body Y of the wheel centres at full bump
    f32 halfTrack = 0.75f;
    WheelParameters wheel;
    bool steered = false;
    // Anti-roll bar: N per metre of compression difference, at the wheels.
    f32 antiRollStiffness = 0.0f;
    // How much of the tyre's lateral force height reaches the body: 1 applies it at the
    // contact patch, 0 at the height of the centre of mass, so the car cannot roll over.
    f32 rollInfluence = 1.0f;
    // Beam axle on a central longitudinal pivot (tractor front axle): rolls +-oscillationLimit
    // relative to the body so both wheels share the load equally; past the limit the stop
    // carries the difference.
    bool oscillating = false;
    f32 oscillationLimit = 0.1745f; // rad (10 degrees)
};

// `maxAngle` is the reference (bicycle-model) angle at the primary steered axle, the
// steered axle farthest from the mean position of the non-steering axles (reference line).
// ackermann = 1: every steered wheel points at one turning centre on that line,
// tan(delta) = d / (R +- lateral offset); 0: wheels of an axle steer in parallel. Further
// steered axles steer by their own distance d to the line. The reference angle slews at
// no more than `rate`.
struct SteeringParameters
{
    f32 maxAngle = 0.6f;
    f32 rate = 1.5f; // rad/s
    f32 ackermann = 1.0f;
};

// Pedals and lever 0..1; steering -1..1, +1 turning right (heading increasing, as for Boat
// and Aircraft); clutch is the PEDAL: 1 pressed (open), 0 released (engaged).
struct VehicleControls
{
    f32 throttle = 0.0f;
    f32 brake = 0.0f;
    f32 handbrake = 0.0f;
    f32 steering = 0.0f;
    f32 clutch = 0.0f;
    bool starter = false;
};

// One wheel as the last applyForces() left it. Forces in the contact frame (Fx along the
// wheel heading, Fy to its left), N; Mz about the normal.
struct WheelTelemetry
{
    bool inContact = false;
    Math::vec3 contactPoint{0.0f};
    Math::vec3 contactNormal{0.0f, 1.0f, 0.0f};
    u32 groundBody = 0xFFFFFFFFu;
    SurfaceInfo surface;
    f32 load = 0.0f;
    f32 longitudinalForce = 0.0f;
    f32 lateralForce = 0.0f;
    f32 aligningMoment = 0.0f;
    // Transient slips (tyre model inputs) and kinematic ones from the wheel speeds,
    // (omega r - vx) / max(|vx|, 1 m/s) and atan(-vy / max(|vx|, 1 m/s)).
    f32 slipRatio = 0.0f;
    f32 slipAngle = 0.0f;
    f32 kinematicSlipRatio = 0.0f;
    f32 kinematicSlipAngle = 0.0f;
    f32 angularVelocity = 0.0f; // rad/s, positive rolling forward
    f32 rotation = 0.0f;        // accumulated spin angle, rad
    f32 steerAngle = 0.0f;      // rad, positive steered right
    f32 compression = 0.0f;     // m from full droop
    f32 suspensionForce = 0.0f; // along the strut, N
    f32 brakeTorque = 0.0f;     // capacity applied this step, N m
    f32 driveTorque = 0.0f;     // from the drivetrain, N m
    bool locked = false;        // braked to a standstill relative to the hub
    Math::vec3 centre{0.0f};    // world
// Columns: axle (left), up, heading; spin included.
Math::mat4 worldTransform{1.0f};
};

struct VehicleTelemetry
{
    f32 forwardSpeed = 0.0f; // m/s along the body's +Z
    f32 lateralSpeed = 0.0f; // m/s, positive to the right
    f32 groundSpeed = 0.0f;
    // Heading turning right from world +Z; roll positive right side down;
    // pitch positive nose up; yaw rate positive turning right. Rad, rad/s.
    f32 heading = 0.0f;
    f32 roll = 0.0f;
    f32 pitch = 0.0f;
    f32 yawRate = 0.0f;
    // Steady-state estimate, yaw rate times forward speed: m/s^2, positive
    // to the right (towards the centre of a right turn).
    f32 lateralAcceleration = 0.0f;
    f32 engineRpm = 0.0f;
    f32 engineTorque = 0.0f; // combustion, N m
    bool engineRunning = false;
    f32 gearboxInputRpm = 0.0f;
    s32 gear = 0;
    bool shifting = false;
    f32 clutchEngagement = 0.0f;
    f32 clutchTorque = 0.0f;
    f32 clutchSlip = 0.0f; // rad/s, engine minus gearbox input
    bool clutchLocked = false;
    f32 steeringAngle = 0.0f; // reference angle, rad
    // Of each oscillating axle in turn (rad, positive left wheel up).
    f32 axleRoll[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    u32 solverRows = 0;
    u32 solverIterations = 0;
    Math::vec3 force{0.0f};  // on the body, world
    Math::vec3 torque{0.0f}; // about the centre of mass, world
};

// Wheeled vehicle on a rigid body; like Aircraft and Boat it does not own the body or add
// a shape (give the body the shape it collides with, above the wheels). Body axes: +Z
// forward, +Y up, right on -X, origin at the centre of mass.
//
// Timing as Aircraft/Boat: call applyForces(step) just before each PhysicsWorld::step(),
// or register stepCallback with setStepCallback() and call applyForces() once by hand
// before the first update(). The forces it adds are consumed by the next integration.
//
// Solve: each wheel casts one ray down its strut; each grounded wheel adds three rows to
// one velocity solve over the chassis (6 DOF) and every shaft (engine, gearbox input,
// differential inputs, wheel spins):
//  - suspension: spring-damper linearised at the end of the step (backward Euler), no pull;
//  - tyre forces from a carcass deflection u per direction, du/dt = -(sliding velocity)
//    - |vx| u / sigma, evaluated at the transient slips kappa' = u_x / sigma_x and
//    tan(alpha') = u_y / sigma_y. At speed this is the first-order slip lag; at a
//    standstill it is a spring, so a parked vehicle holds on a slope. Force = secant
//    stiffness times end-of-step deflection (implicit, stable at 60 Hz) inside the
//    friction ellipse. u is advanced at the start of each step with the slip the patch
//    really had after the world's joints and contacts, so a trailer hitch pushing a braked
//    tractor winds the tyres up until they hold it;
//  - wheel friction: brake, handbrake, rolling resistance and bearing drag as one Coulomb
//    element bounded by their sum, so a braked wheel locks at exactly zero speed.
// The drivetrain adds engine friction (bounded), clutch (bounded by capacity), gear mesh and
// differential kinematics (rigid), lockers and limited-slip clutches (bounded), viscous
// couplings (soft). Gravity and forces already on the body are predicted into the free
// velocities. The solve is direct (Cholesky in double, see VehicleSolver) with an active set
// for the bounds, warm-started; tyre limits use last step's loads and the solve repeats if
// the loads differ by more than 5%. The net chassis impulse is applied as one force and torque.
class GroundVehicle
{
public:
    // Asked once per grounded wheel per step; null keeps the default surface.
    using SurfaceCallback = SurfaceInfo (*)(u32 body, const Math::vec3& point, void* userData);

    GroundVehicle();
    ~GroundVehicle();
    GroundVehicle(const GroundVehicle&) = delete;
    GroundVehicle& operator=(const GroundVehicle&) = delete;

    // The body and the id the world gave it (the rays ignore it).
    void setBody(RigidBody* body, u32 bodyId);
    RigidBody* body() const
    {
        return mBody;
    }
    u32 bodyId() const
    {
        return mBodyId;
    }
    void setWorld(PhysicsWorld* world);
    PhysicsWorld* world() const
    {
        return mWorld;
    }
    // The filter for the wheel rays; its ignoredBody is always this vehicle's body. To keep a
    // coupled vehicle's rays off the other's body, put both in a group this filter excludes.
    void setRayFilter(const QueryFilter& filter);
    void setSurfaceCallback(SurfaceCallback callback, void* userData);
    void setDefaultSurface(const SurfaceInfo& surface)
    {
        mDefaultSurface = surface;
    }

    // Returns the axle index; its wheels are 2 * index and 2 * index + 1.
    u32 addAxle(const AxleParameters& axle);
    u32 axleCount() const
    {
        return static_cast<u32>(mAxles.size());
    }
    // Editable between steps (anti-roll bars, brakes, tyres...).
    AxleParameters& axle(u32 index)
    {
        return mAxles[index];
    }
    const AxleParameters& axle(u32 index) const
    {
        return mAxles[index];
    }
    u32 wheelCount() const
    {
        return static_cast<u32>(mWheels.size());
    }
    const WheelTelemetry& wheel(u32 index) const
    {
        return mWheels[index].telemetry;
    }
    // The wheel's spin, rad/s; set it to start a vehicle already rolling.
    void setWheelAngularVelocity(u32 index, f32 omega);
    // Every wheel at rest and, in a forward gear, the first one in mesh: for a vehicle that is
    // put down somewhere at rest.
    void stopWheels();
    // A wheel that has come off: it no longer touches the ground, carries no load and gives no
    // grip, and the suspension at that corner is gone. The drivetrain still turns it.
    void setWheelDetached(u32 index, bool detached);
    // A tyre that has burst: the wheel stands on its rim, so the radius it rolls and the ground it
    // finds are that fraction of the axle's.
    void setWheelRadiusScale(u32 index, f32 scale);
    f32 wheelRadiusScale(u32 index) const
    {
        return index < mWheels.size() ? mWheels[index].radiusScale : 1.0f;
    }
    bool wheelDetached(u32 index) const
    {
        return index < mWheels.size() && mWheels[index].detached;
    }

    void setSteering(const SteeringParameters& steering);
    const SteeringParameters& steering() const
    {
        return mSteering;
    }
    // Distance from the reference line to the primary steered axle - the
    // wheelbase of the equivalent bicycle model - and that line's body Z.
    f32 wheelbase() const;
    f32 referenceAxlePosition() const;
    f32 wheelSteerAngle(u32 index, f32 referenceAngle) const;

    // Without a root differential (the default) the vehicle is unpowered (a trailer) and the
    // engine, clutch and gearbox are ignored.
    Engine& engine()
    {
        return mEngine;
    }
    const Engine& engine() const
    {
        return mEngine;
    }
    Clutch& clutch()
    {
        return mClutch;
    }
    const Clutch& clutch() const
    {
        return mClutch;
    }
    Gearbox& gearbox()
    {
        return mGearbox;
    }
    const Gearbox& gearbox() const
    {
        return mGearbox;
    }
    // Outputs may name wheels or differentials added later; they are
    // checked when the vehicle first steps.
    u32 addDifferential(const DifferentialParameters& parameters, DriveOutput a, DriveOutput b);
    u32 differentialCount() const
    {
        return static_cast<u32>(mDifferentials.size());
    }
    Differential& differential(u32 index)
    {
        return mDifferentials[index];
    }
    const Differential& differential(u32 index) const
    {
        return mDifferentials[index];
    }
    // The differential the gearbox output drives; -1 for none.
    void setDriveRoot(s32 differential);
    s32 driveRoot() const
    {
        return mRoot;
    }
    bool hasDrivetrain() const;

    // Automatic clutch for a manual box (always on for an automatic one). It engages with the
    // faster of engine and gearbox input, from nothing at engageRpm to full at a speed rising
    // with throttle from fullRpm to launchRpm (no stall, hard launches at high revs, a gear
    // selected on the move engages at once); once the gearbox input alone exceeds engageRpm the
    // rest follows over about a second. It opens for every shift and ramps closed over
    // engageTime, the throttle lifted for the shift. The pedal can still open it further.
    // Speeds <= 0 pick defaults from the engine's idle and limiter speeds.
    void setAutoClutch(bool enabled)
    {
        mAutoClutch = enabled;
    }
    bool autoClutch() const
    {
        return mAutoClutch;
    }
    void setAutoClutchSpeeds(f32 engageRpm, f32 fullRpm, f32 launchRpm, f32 engageTime = 0.35f);

    // Anti-lock brakes: a wheel's brake pressure is released while its
    // kinematic slip ratio is below -absSlip at more than 2 m/s, and
    // reapplied above half of it.
    void setAntiLockBrakes(bool enabled, f32 absSlip = 0.12f);

    // Air drag 0.5 rho CdA |v| v at a body-space centre of pressure.
    void setAerodynamics(f32 dragArea, const Math::vec3& centreOfPressure = Math::vec3(0.0f),
                         f32 airDensity = 1.225f);

    void setControls(const VehicleControls& controls);
    const VehicleControls& controls() const
    {
        return mControls;
    }
    void shiftUp();
    void shiftDown();
    // -1 reverse, 0 neutral, 1.. forward (in automatic, any forward gear is
    // "drive").
    void selectGear(s32 gear);
    void setDifferentialLock(u32 differential, bool locked);
    void toggleDifferentialLock(u32 differential);

    // The trailer side of a brake line: this vehicle's brake follows the
    // other's brake pedal (its own controls().brake is ignored while set).
    // Null disconnects.
    void setBrakeSource(const GroundVehicle* source)
    {
        mBrakeSource = source;
    }
    const GroundVehicle* brakeSource() const
    {
        return mBrakeSource;
    }
    f32 appliedBrake() const
    {
        return mAppliedBrake;
    }

    void applyForces(f32 step);
    // For PhysicsWorld::setStepCallback, with the GroundVehicle as userData.
    static void stepCallback(f32 step, void* userData);

    const VehicleTelemetry& telemetry() const
    {
        return mTelemetry;
    }

private:
    struct WheelState
    {
        u32 axle = 0;
        f32 side = 1.0f; // +1 left, -1 right
        f32 omega = 0.0f;
        f32 rotation = 0.0f;
        // Carcass deflection at the start of the step; advanced at the start of the next step
        // from the motion the world actually produced.
        f32 deflectionX = 0.0f;
        f32 deflectionY = 0.0f;
        f32 lastRelaxX = 1.0f;
        f32 lastRelaxY = 1.0f;
        f32 lastStep = 0.0f;
        bool wasGrounded = false;
        bool detached = false;
        f32 radiusScale = 1.0f;
        f32 absPressure = 1.0f;
        f32 lastLoad = 0.0f;
        WheelTelemetry telemetry;

        bool grounded = false;
        bool bottomed = false;
        f32 length = 0.0f;
        f32 compression = 0.0f;
        f32 cosine = 1.0f;
        Math::vec3 hardPoint{0.0f};
        Math::vec3 down{0.0f, -1.0f, 0.0f};
        Math::vec3 heading{0.0f, 0.0f, 1.0f};
        Math::vec3 point{0.0f};
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        Math::vec3 groundVelocity{0.0f};
        Math::vec3 forward{0.0f, 0.0f, 1.0f};
        Math::vec3 lateral{1.0f, 0.0f, 0.0f};
        f32 camber = 0.0f;
        f32 vx = 0.0f;
        f32 vy = 0.0f;
        f32 strutForce = 0.0f;
        f32 strutStiffness = 0.0f;
        f32 strutDamping = 0.0f;
        f32 loadEstimate = 0.0f;
        u32 dof = 0;
        s32 normalRow = -1;
        s32 rowX = -1;
        s32 rowY = -1;
        s32 frictionRow = -1;
        f32 brakeTorque = 0.0f;
    };

    void validateDrivetrain();
    void updateSteering(f32 step);
    void probeWheels(f32 step);
    void shareOscillatingAxles();
    void computeStruts();
    void updateClutchAndGearbox(f32 step, f32 throttle);
    void buildRows(f32 step, bool firstPass);
    void updateTyreRows(f32 step, bool firstPass);
    void finishStep(f32 step);
    void updateTelemetry(const Math::vec3& force, const Math::vec3& torque);
    f32 noSlipVelocity(DriveOutput output, u32 depth) const;
    bool drivenGrounded(DriveOutput output, u32 depth) const;
    bool drivenSlipping(DriveOutput output, u32 depth) const;

    RigidBody* mBody = nullptr;
    u32 mBodyId = 0xFFFFFFFFu;
    PhysicsWorld* mWorld = nullptr;
    QueryFilter mRayFilter;
    SurfaceCallback mSurfaceCallback = nullptr;
    void* mSurfaceUserData = nullptr;
    SurfaceInfo mDefaultSurface;

    ct::Vector<AxleParameters> mAxles;
    ct::Vector<WheelState> mWheels;
    ct::Vector<Tyre> mTyres; // one per axle, rebuilt when its parameters change
    ct::Vector<TyreParameters> mTyreSource;
    SteeringParameters mSteering;
    f32 mSteerAngle = 0.0f;

    Engine mEngine;
    Clutch mClutch;
    Gearbox mGearbox;
    f32 mRoadSpeed = 0.0f;
    f32 mLastEngineRpm = 0.0f;
    ct::Vector<Differential> mDifferentials;
    s32 mRoot = -1;
    bool mDrivetrainValid = false;
    bool mDrivetrainChecked = false;
    bool mAutoClutch = false;
    f32 mAutoEngageRpm = -1.0f;
    f32 mAutoFullRpm = -1.0f;
    f32 mAutoLaunchRpm = -1.0f;
    f32 mAutoEngageTime = 0.35f;
    f32 mAutoClosing = 0.0f;
    bool mAbs = false;
    f32 mAbsSlip = 0.12f;

    f32 mDragArea = 0.0f;
    f32 mAirDensity = 1.225f;
    Math::vec3 mCentreOfPressure{0.0f};

    VehicleControls mControls;
    const GroundVehicle* mBrakeSource = nullptr;
    f32 mAppliedBrake = 0.0f;
    VehicleTelemetry mTelemetry;

    VehicleSolver* mSolver = nullptr;
    u32 mEngineDof = 0;
    u32 mInputDof = 0;
    u32 mDifferentialDof = 0;
    s32 mEngineFrictionRow = -1;
    s32 mClutchRow = -1;
    s32 mGearRow = -1;
    s32 mGearboxLossRow = -1;
    ct::Vector<s32> mDifferentialRows; // kinematic row per differential
    ct::Vector<s32> mLockRows;         // locking element row per differential
    ct::Vector<s32> mPairRows;         // shared normal row per oscillating axle
    f32 mCombustion = 0.0f;
    Math::vec3 mAeroForce{0.0f};
    Math::vec3 mAeroTorque{0.0f};
    ct::Vector<u32> mLastKeys;
    ct::Vector<f32> mLastImpulses;
    f32 mLastStep = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_GROUNDVEHICLE_H
