#ifndef KINETIX_PHYSICS_MACHINERY_TRACKEDUNDERCARRIAGE_H
#define KINETIX_PHYSICS_MACHINERY_TRACKEDUNDERCARRIAGE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "collision/CollisionFilter.h"
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"
#include "machinery/Terramechanics.h"

namespace kinetix::Physics
{

class PhysicsWorld;
class RigidBody;

// Geometry and running gear of a pair of tracks in the hull's body frame (+Z forward, +Y up,
// right -X; the left track at +gauge / 2).
struct TrackParameters
{
    // Centre to centre of the two tracks; shoe width b, m.
    f32 gauge = 2.0f;
    f32 shoeWidth = 0.6f;
    // Bottom run on the ground (first road wheel to last) and its middle along the hull (z), m.
    f32 contactLength = 3.6f;
    f32 contactCentre = 0.0f;
    // Height of the track's ground face (bottom of the shoes) in the hull frame, m.
    f32 bottomHeight = -0.8f;
    // Running gear (drive radius and drawing): sprocket at the rear and idler at the front (z of
    // centres, radii), road wheels on the bottom run.
    f32 sprocketRadius = 0.35f;
    f32 sprocketZ = -2.2f;
    f32 idlerRadius = 0.32f;
    f32 idlerZ = 2.2f;
    u32 roadWheels = 7;
    f32 roadWheelRadius = 0.1f;
    // Contact samples along each bottom run, over which pressure and shear are integrated.
    u32 samples = 12;
    // Each track's circulating parts as a mass at the belt, kg.
    f32 beltMass = 1200.0f;
    // Internal motion resistance as a fraction of the track's normal load, opposing the belt's run.
    f32 internalResistance = 0.03f;
    // Soil contact damping, ratio of critical for the supported mass shared by the samples in contact.
    f32 soilDampingRatio = 0.8f;
    // Mass the tracks carry (the whole machine), kg, for that damping. 0: the hull's own.
    f32 supportedMass = 0.0f;
    // Ground probes start this far above the ground face and reach this far below it, m.
    f32 probeHeight = 0.6f;
    f32 probeDepth = 0.8f;
};

// A pair of tracks under a hull as continuous belts on the ground: a Joint on the hull with one
// degree of freedom per track (the belt's speed), solved with the world.
//
// Per sample of each bottom run (TrackParameters::samples, even over the contact length), every step:
//  - a probe down the hull's -Y finds the ground and normal; the sinkage z gives Bekker's pressure
//    p = (kc / b + kphi) z^n (stiff contact on a hard surface) as force p b dl, solved implicitly
//    (linearised about z, with soil damping) so the machine settles and stays stable at 60 Hz;
//  - the shear displacement j (2D in the ground plane: along and across the track) of the belt
//    against the soil: dj/dt = belt slip velocity, advected with the belt (semi-Lagrangian: each
//    sample reads the displacement of the soil now under it from where that soil was a step ago).
//    In steady slip i, j = i x exactly (x from the front); at standstill it is a spring holding
//    the machine on a slope. Shear stress is Janosi-Hanamoto's
//    tau = (c + sigma tan phi)(1 - exp(-|j| / K)) along -j, also implicit (tangent stiffness about
//    j), capped at c + sigma tan phi (mu sigma on a hard surface);
//  - Bekker's compaction resistance b (kc / b + kphi) z^(n+1) / (n + 1) per track, a bounded
//    resistance to the hull's motion, and the internal resistance on the belt.
//
// Integrated over the run that is the tractive effort H(i) = (A c + W tan phi)[1 - K / (i L)(1 -
// exp(-i L / K))]; the drawbar pull is H less the resistances. Skid steering by the two belt
// speeds: opposite speeds turn on the spot against the lateral shear of the run.
//
// drive(side) is a JointDrive whose coordinate is the belt's travel (m, + forward), velocity
// mode: driven by a HydraulicMotor with outputRadius = sprocket radius, or set directly. A belt
// with no drive rolls free.
//
// Add it to the world like any joint; the hull body is both of its bodies. The ground is
// whatever the probes hit that rayFilter accepts (static or kinematic; a dynamic ground body
// feels the reaction a step later), with the soil set for it (setSoil) or the default one.
class TrackedUndercarriage final : public Joint
{
public:
    static constexpr u32 MaxSamples = 32;
    static constexpr u32 MaxSoilOverrides = 8;
    static constexpr u32 Left = 0;
    static constexpr u32 Right = 1;

    TrackedUndercarriage(PhysicsWorld& world, u32 hullBodyId, const TrackParameters& parameters);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    bool singleBody() const override
    {
        return true;
    }
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    const TrackParameters& parameters() const
    {
        return mParameters;
    }
    void setSoil(const SoilParameters& soil);
    bool setSoil(u32 groundBodyId, const SoilParameters& soil);
    const SoilParameters& soil() const
    {
        return mSoil;
    }
    void setRayFilter(const QueryFilter& filter);

    JointDrive& drive(u32 side)
    {
        return mSides[side].drive;
    }
    const JointDrive& drive(u32 side) const
    {
        return mSides[side].drive;
    }

    struct SideState
    {
        // Belt speed (m/s, + forward) and travel (m); the hull's speed over the ground along the track, m/s.
        f32 beltSpeed = 0.0f;
        f32 beltTravel = 0.0f;
        f32 groundSpeed = 0.0f;
        // Slip i = 1 - ground / belt speed (0 when the belt is still).
        f32 slip = 0.0f;
        // Ground forces on the track: along it (tractive effort, N), across it, normal load,
        // compaction resistance, internal resistance (N).
        f32 thrust = 0.0f;
        f32 lateralForce = 0.0f;
        f32 normalLoad = 0.0f;
        f32 compactionResistance = 0.0f;
        f32 internalResistance = 0.0f;
        f32 meanSinkage = 0.0f;
        f32 maxSinkage = 0.0f;
        u32 samplesInContact = 0;
        f32 driveForce = 0.0f;
    };
    struct SampleState
    {
        Math::vec3 point{0.0f};
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        f32 sinkage = 0.0f;
        f32 pressure = 0.0f;
        // Shear displacement along and across the track, m; shear stress on the shoe, Pa (along, across).
        Math::vec2 shearDisplacement{0.0f};
        Math::vec2 shearStress{0.0f};
        bool contact = false;
    };
    const SideState& side(u32 index) const
    {
        return mSides[index].state;
    }
    const SampleState& sample(u32 side, u32 index) const
    {
        return mSides[side].samples[index].state;
    }
    u32 sampleCount() const
    {
        return mSampleCount;
    }
    Math::vec3 sampleLocal(u32 side, u32 index) const;
    // Total ground force and moment (about the hull's centre of mass) applied last step, world, N and N*m.
    const Math::vec3& groundForce() const
    {
        return mGroundForce;
    }

private:
    struct Row
    {
        Math::vec3 linear{0.0f};
        Math::vec3 angular{0.0f};
        f32 belt = 0.0f;
        f32 inverseMass = 0.0f;
        // Soft rows: lambda = softA - softB * u; rigid ones aim u at 0.
        f32 softA = 0.0f;
        f32 softB = 0.0f;
        f32 lo = 0.0f;
        f32 hi = 0.0f;
        f32 impulse = 0.0f;
    };
    struct Sample
    {
        SampleState state;
        u32 groundBody = 0xFFFFFFFFu;
        bool active = false;
        Math::vec3 forward{0.0f, 0.0f, 1.0f};
        Math::vec3 lateral{1.0f, 0.0f, 0.0f};
        // Shear displacement carried in from upstream; share of the step this sample's soil has been under the track.
        Math::vec2 advected{0.0f};
        f32 slipTime = 0.0f;
        // Slip velocity the step ended with (along, across), m/s.
        Math::vec2 lastSlip{0.0f};
        f32 shearCap = 0.0f;
        const SoilParameters* soil = nullptr;
        Row normal;
        Row along;
        Row across;
    };
    struct Side
    {
        JointDrive drive;
        f32 beltSpeed = 0.0f;
        f32 beltTravel = 0.0f;
        Sample samples[MaxSamples];
        Row compaction;
        Row internal;
        bool driveActive = false;
        SideState state;
    };

    const SoilParameters& soilFor(u32 bodyId) const;
    void probe(Side& side, u32 sideIndex, f32 duration);
    void advance(Side& side, f32 duration);
    void buildRow(Row& row, const Math::vec3& point, const Math::vec3& direction, f32 belt) const;
    f32 rowVelocity(const Row& row, f32 beltSpeed) const;
    void applyRow(Row& row, Side& side, f32 impulse);
    void solveSoft(Row& row, Side& side);
    void solveBounded(Row& row, Side& side);
    void finishStep();

    PhysicsWorld* mWorld;
    u32 mHullId;
    RigidBody* mHull;
    TrackParameters mParameters;
    SoilParameters mSoil;
    u32 mOverrideBodies[MaxSoilOverrides] = {};
    SoilParameters mOverrideSoils[MaxSoilOverrides];
    u32 mOverrideCount = 0;
    QueryFilter mRayFilter;
    u32 mSampleCount = 12;
    f32 mSampleLength = 0.3f;
    Side mSides[2];
    f32 mDuration = 0.0f;
    f32 mPreviousDuration = 0.0f;
    bool mStepped = false;
    Math::vec3 mGroundForce{0.0f};
    Math::vec3 mGroundTorque{0.0f};
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_TRACKEDUNDERCARRIAGE_H
