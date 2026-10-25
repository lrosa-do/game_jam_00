#ifndef KINETIX_PHYSICS_MACHINERY_BULLDOZER_H
#define KINETIX_PHYSICS_MACHINERY_BULLDOZER_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/HydraulicCylinder.h"
#include "actuators/HydraulicMotor.h"
#include "collision/CollisionFilter.h"
#include "collision/CollisionShape.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/RigidBody.h"
#include "machinery/HydraulicSystem.h"
#include "machinery/SoilCutting.h"
#include "machinery/TrackedUndercarriage.h"

namespace kinetix::Physics
{

class PhysicsWorld;

// Crawler dozer in the tractor's body frame (+Z forward, +Y up, left +X), SI; side-view points
// are (z, y) from the tractor's centre of mass. The push frame pivots on trunnions on the track
// frames; the blade tilts on a hinge along the machine. Lift cylinders (ganged) raise the push
// frame; the tilt cylinder tilts the blade's left side.
struct BulldozerParameters
{
    Math::vec3 tractorHalfExtents{1.0f, 0.55f, 2.2f};
    f32 tractorMass = 15000.0f;
    // Centre of mass height over the track's ground face.
    f32 rideHeight = 1.0f;

    f32 trunnionZ = -0.3f;
    f32 trunnionY = -0.6f;
    f32 tiltHingeZ = 3.0f;
    f32 tiltHingeY = -0.55f;
    f32 pushFrameHalfWidth = 1.15f;
    f32 pushFrameHalfThickness = 0.12f;
    f32 pushFrameMass = 1600.0f;

    // bladeForward: centre from the tilt hinge; cuttingAngle to the ground; buildEdgeHeight: edge
    // above the track's ground face in the build pose.
    f32 bladeWidth = 3.4f;
    f32 bladeHeight = 1.3f;
    f32 bladeThickness = 0.2f;
    f32 bladeMass = 1900.0f;
    f32 bladeForward = 0.15f;
    f32 cuttingAngle = 0.96f;
    f32 buildEdgeHeight = 0.05f;

    // Lift: (z, y) barrel on the tractor, rod on the push frame; tilt: (x, z, y) on the push frame and the blade.
    f32 liftBarrelZ = 1.9f;
    f32 liftBarrelY = 0.45f;
    f32 liftRodZ = 2.7f;
    f32 liftRodY = -0.45f;
    Math::vec3 tiltBarrel{1.1f, -0.5f, 2.2f};
    Math::vec3 tiltRod{1.1f, 0.3f, 2.95f};
    HydraulicCylinderParameters liftCylinder;
    HydraulicCylinderParameters tiltCylinder;

    TrackParameters tracks;
    EngineParameters engine;
    // One load-sensing pump for the blade, one per track for travel.
    HydraulicPumpParameters implementPump;
    HydraulicPumpParameters travelPump;
    HydraulicMotorParameters travelMotor;
    ValveSectionParameters liftSection;
    ValveSectionParameters tiltSection;
    ValveSectionParameters travelSection;

    // Bodies collide with bodyFilter (not each other); the blade skips diggableGroup (a soil
    // bed's group) so its edge can enter the soil.
    CollisionFilter bodyFilter{1u << 6, ~(1u << 6)};
    u32 diggableGroup = 1u << 7;

    // 18 t class: 150 kW, 3.4 m x 1.3 m straight tilt blade, hydrostatic travel to 3.6 km/h.
    static BulldozerParameters crawler18t();
};

// The operator's levers, -1..1 each.
struct BulldozerControls
{
    // Travel: + forward, each track.
    f32 leftTrack = 0.0f;
    f32 rightTrack = 0.0f;
    // + raises the blade; + tilts it with its left end up.
    f32 lift = 0.0f;
    f32 tilt = 0.0f;
    // Engine speed dial, rpm (0: the engine's rated speed).
    f32 engineRpm = 0.0f;
};

// A crawler dozer built into a world. The blade's cutting edge is a SoilCuttingTool (soil bed
// cutting resistance); whatever the blade's box touches it pushes by contact.
// The machine owns its bodies, shapes and joints; the world does not.
// Call update(step) before every world step.
class Bulldozer
{
public:
    Bulldozer(PhysicsWorld& world, const BulldozerParameters& parameters, const Math::vec3& position,
              const Math::quat& orientation = Math::quat());
    ~Bulldozer();
    Bulldozer(const Bulldozer&) = delete;
    Bulldozer& operator=(const Bulldozer&) = delete;

    void setControls(const BulldozerControls& controls);
    const BulldozerControls& controls() const
    {
        return mControls;
    }
    // The soil the blade's edge cuts and the tracks stand on.
    void setSoilBed(const SoilBed& bed);
    void setGroundSoil(const SoilParameters& soil);

    void update(f32 step);

    static constexpr u32 BodyCount = 3;
    enum BodyIndex : u32
    {
        Tractor,
        PushFrame,
        Blade
    };
    RigidBody& tractor()
    {
        return mBodies[Tractor];
    }
    RigidBody& pushFrame()
    {
        return mBodies[PushFrame];
    }
    RigidBody& blade()
    {
        return mBodies[Blade];
    }
    const RigidBody& body(u32 index) const
    {
        return mBodies[index];
    }
    const BoxShape& shape(u32 index) const
    {
        return *mShapes[index];
    }
    u32 bodyId(u32 index) const
    {
        return mIds[index];
    }
    HingeJoint& trunnion()
    {
        return *mTrunnion;
    }
    HingeJoint& tiltHinge()
    {
        return *mTiltHinge;
    }
    HydraulicCylinder& liftCylinder()
    {
        return *mLiftCylinder;
    }
    HydraulicCylinder& tiltCylinder()
    {
        return *mTiltCylinder;
    }
    HydraulicMotor& travelMotor(u32 side)
    {
        return *mTravelMotors[side];
    }
    HydraulicSystem& hydraulics()
    {
        return *mHydraulics;
    }
    TrackedUndercarriage& tracks()
    {
        return *mTracks;
    }
    SoilCuttingTool& cuttingEdge()
    {
        return *mEdge;
    }
    const BulldozerParameters& parameters() const
    {
        return mParameters;
    }

    Math::vec3 edgeWorld() const;
    // m, + above the plane of the tracks' ground face.
    f32 edgeHeight() const;
    // rad, + left end up.
    f32 bladeTilt() const;
    // Force on the blade from the last step, N, world frame: contacts plus soil cutting resistance.
    Math::vec3 bladeForce() const;
    f32 worstJointSeparation() const;
    bool finite() const;

    enum Section : u32
    {
        LiftSection,
        TiltSection,
        LeftTravelSection,
        RightTravelSection,
        SectionCount
    };
    u32 section(Section which) const
    {
        return mSections[which];
    }

private:
    Math::vec3 frameWorld(f32 x, f32 z, f32 y) const;
    void addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation, const Math::vec3& halfExtents,
                 f32 mass, const CollisionFilter& filter);

    PhysicsWorld* mWorld;
    BulldozerParameters mParameters;
    BulldozerControls mControls;
    Math::vec3 mPosition{0.0f};
    Math::quat mOrientation;

    RigidBody mBodies[BodyCount];
    BoxShape* mShapes[BodyCount] = {};
    u32 mIds[BodyCount] = {};

    HingeJoint* mTrunnion = nullptr;
    HingeJoint* mTiltHinge = nullptr;
    HydraulicCylinder* mLiftCylinder = nullptr;
    HydraulicCylinder* mTiltCylinder = nullptr;
    HydraulicMotor* mTravelMotors[2] = {};
    HydraulicSystem* mHydraulics = nullptr;
    TrackedUndercarriage* mTracks = nullptr;
    SoilCuttingTool* mEdge = nullptr;
    u32 mSections[SectionCount] = {};

    Math::vec3 mPins[2][2];
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_BULLDOZER_H
