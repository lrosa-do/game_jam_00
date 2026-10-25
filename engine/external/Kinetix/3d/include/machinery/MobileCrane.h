#ifndef KINETIX_PHYSICS_MACHINERY_MOBILECRANE_H
#define KINETIX_PHYSICS_MACHINERY_MOBILECRANE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/HydraulicCylinder.h"
#include "actuators/HydraulicMotor.h"
#include "actuators/Hook.h"
#include "actuators/Rope.h"
#include "collision/CollisionFilter.h"
#include "collision/CollisionShape.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/RigidBody.h"
#include "dynamics/SliderJoint.h"
#include "machinery/AntiSway.h"
#include "machinery/HydraulicSystem.h"
#include "machinery/LoadMomentIndicator.h"

namespace kinetix::Physics
{

class PhysicsWorld;

// An all-terrain mobile crane in its carrier's body frame (+Z forward, +Y up, right -X), SI. The
// carrier is a rigid box on the ground (running gear folded into it) until the outriggers lift
// it: four beams sliding out sideways, each with a jack sliding down onto a pad. The
// superstructure slews on the carrier's rear (counterweight, winch, boom): a luffing base
// section with telescopic sections sliding out of it, the head sheave on the last, the rope
// reeved to a hook block.
struct MobileCraneParameters
{
    // Carrier chassis: half extents; centre of mass height over the ground (standing, its box
    // reaches the ground); mass, running gear included.
    Math::vec3 chassisHalfExtents{1.25f, 0.8f, 6.0f};
    f32 chassisHeight = 0.8f;
    f32 chassisMass = 34000.0f;

    // Outriggers: beam z (front, rear); pad centre |x| fully out and retracted; beams' height from
    // the chassis centre; masses; jack stroke; pad height above ground fully retracted.
    f32 outriggerZ[2] = {3.4f, -4.6f};
    f32 outriggerSpread = 3.6f;
    f32 outriggerRetracted = 1.3f;
    f32 beamHeight = -0.3f;
    f32 beamMass = 600.0f;
    f32 jackMass = 400.0f;
    f32 jackStroke = 1.4f;
    f32 padClearance = 0.3f;
    f32 beamForce = 1.5e5f;
    f32 jackForce = 8.0e5f;
    f32 beamSpeed = 0.3f;
    f32 jackSpeed = 0.15f;

    // Superstructure: slewing axis z on the chassis; its centre of mass (counterweight included),
    // box and mass.
    f32 slewZ = -2.6f;
    Math::vec3 superHalfExtents{1.25f, 0.6f, 2.0f};
    Math::vec3 superCentre{0.0f, 0.6f, -1.4f};
    f32 superMass = 22000.0f;

    // Boom: foot pin over the slewing plane (forward, up); base section length, each telescopic
    // section's length, masses, maximum slide-out; build elevation.
    f32 boomFootZ = 0.4f;
    f32 boomFootHeight = 1.8f;
    f32 baseLength = 11.0f;
    f32 sectionLength = 10.5f;
    static constexpr u32 SectionCount = 3;
    f32 baseMass = 4500.0f;
    f32 sectionMass[SectionCount] = {2600.0f, 1800.0f, 1200.0f};
    f32 sectionExtension = 8.5f;
    f32 buildElevation = 0.17f;
    // Telescoping force; it also holds the sections out against the load's thrust and the rope's pull.
    f32 telescopeForce = 2.5e6f;
    f32 telescopeSpeed = 0.25f;
    // Luffing cylinder: from the superstructure (forward, up over the slewing plane) to the base
    // section (along it, below its axis).
    f32 luffBarrelZ = 1.6f;
    f32 luffBarrelHeight = 0.5f;
    f32 luffRodAlong = 4.2f;
    f32 luffRodBelow = 0.6f;
    HydraulicCylinderParameters luffCylinder;

    // Hoist: drum on the superstructure, head sheave (radius, efficiency), falls, hook block.
    WinchParameters winch;
    Math::vec3 drumPoint{0.0f, 1.6f, -2.2f};
    f32 sheaveRadius = 0.35f;
    f32 sheaveEfficiency = 0.98f;
    u32 falls = 4;
    f32 hookBlockMass = 500.0f;
    f32 hookDrop = 4.0f;

    EngineParameters engine;
    HydraulicPumpParameters pump;
    HydraulicMotorParameters slewMotor;
    ValveSectionParameters luffSection;
    ValveSectionParameters slewSection;

    // Load chart: rated = this share of the tipping load (75% on outriggers), capped by
    // `structuralMoment` (kg m) of load moment and `maxLoad` at any radius.
    f32 stabilityFactor = 0.75f;
    f32 structuralMoment = 3.0e5f;
    f32 maxLoad = 50000.0f;

    CollisionFilter bodyFilter{1u << 6, ~(1u << 6)};

    // Build pose: outriggers out, carrier lifted `liftHeight` clear of the ground, superstructure
    // slewed by `buildSlew` (rad, + left), telescopic sections out by `buildExtension` (m). Every
    // joint reads zero there except the outrigger slides and telescopes (their travel).
    bool buildDeployed = false;
    f32 liftHeight = 0.15f;
    f32 buildSlew = 0.0f;
    f32 buildExtension = 0.0f;

    // 50 t class: 34 t carrier, 7.2 m outrigger base, 22 t superstructure, 11 to 36.5 m
    // four-section boom, 4 falls.
    static MobileCraneParameters allTerrain50t();
};

struct MobileCraneControls
{
    // + slews left (counter-clockwise from above), + booms up, + telescopes out, + hoists; -1..1.
    f32 slew = 0.0f;
    f32 luff = 0.0f;
    f32 telescope = 0.0f;
    f32 hoist = 0.0f;
    // Outriggers: beams out (0 retracted .. 1 fully out), jacks down (0 .. 1 of stroke).
    f32 outriggerBeams = 0.0f;
    f32 outriggerJacks = 0.0f;
    f32 engineRpm = 0.0f;
};

// A mobile crane built into a world. Every joint of the carrier, outriggers and boom is
// articulated (Joint::setArticulated) so the load on four jacks and the boom stay rigid. The
// load moment indicator rates every lift against the crane's own load chart (computed from its
// masses and outrigger base at construction) and, enabled, cuts out motions that would raise the
// moment past it. The optional anti-sway shaper smooths the slewing lever for the rope's
// pendulum period.
//
// Call update(step) before every world step.
class MobileCrane
{
public:
    // `position`: chassis centre of mass standing on the ground (built deployed, the crane lifts
    // itself from there).
    MobileCrane(PhysicsWorld& world, const MobileCraneParameters& parameters, Math::vec3 position,
                const Math::quat& orientation = Math::quat());
    ~MobileCrane();
    MobileCrane(const MobileCrane&) = delete;
    MobileCrane& operator=(const MobileCrane&) = delete;

    void setControls(const MobileCraneControls& controls);
    const MobileCraneControls& controls() const
    {
        return mControls;
    }
    void setAntiSway(bool enabled);
    bool antiSway() const
    {
        return mAntiSway;
    }
    LoadMomentIndicator& lmi()
    {
        return mLmi;
    }
    const LoadMomentIndicator& lmi() const
    {
        return mLmi;
    }

    void update(f32 step);

    // Boom length (foot pin to head sheave), elevation (rad), radius (slew axis to head sheave,
    // horizontal), m.
    f32 boomLength() const;
    f32 boomElevation() const;
    f32 radius() const;
    f32 hookLoad() const;
    // Each outrigger's jack force, N (+ pushing down): front-left, front-right, rear-left, rear-right.
    f32 outriggerLoad(u32 index) const;
    // N the carrier's own box carries on the ground from the world's contacts: zero once lifted.
    f32 carrierGroundLoad() const;
    f32 tilt() const;
    Math::vec3 slewAxisWorld() const;
    Math::vec3 headSheaveWorld() const;
    // Pendulum length: head sheave to the hook block's centre of mass.
    f32 ropeLength() const;
    // Swing angle of the hook block below the head sheave, rad.
    f32 swingAngle() const;
    bool finite() const;
    f32 worstJointSeparation() const;

    // Head sheave in the build pose, in the carrier's frame from the chassis' centre of mass
    // standing on the ground (lifted by liftHeight when built deployed).
    static Math::vec3 buildHeadLocal(const MobileCraneParameters& parameters);
    // The crane's own chart: rated load (kg) by boom length and radius, over the side on
    // outriggers (360 deg chart).
    static LoadChart computeChart(const MobileCraneParameters& parameters);
    // Moment about the tipping line over the side (N*m, + holding the crane down) of the crane
    // alone with this boom, slewed square to the carrier: the chart's basis; a load at radius r
    // takes away (r - spread) g per kg.
    static f32 restoringMoment(const MobileCraneParameters& parameters, f32 boomLength, f32 radius);

    RigidBody& chassis()
    {
        return mBodies[Chassis];
    }
    RigidBody& superstructure()
    {
        return mBodies[Superstructure];
    }
    RigidBody& boomSection(u32 index)
    {
        return mBodies[BoomBase + index];
    }
    RigidBody& hookBlock()
    {
        return mBodies[HookBlock];
    }
    RigidBody& jack(u32 index)
    {
        return mBodies[Jack0 + index];
    }
    RigidBody& beam(u32 index)
    {
        return mBodies[Beam0 + index];
    }
    const RigidBody& body(u32 index) const
    {
        return mBodies[index];
    }
    // Collision shape: boxes, but spheres of the jack's half height for the jacks (a pad on its
    // ball seat transmits force, not moment).
    const CollisionShape& shape(u32 index) const
    {
        return *mShapes[index];
    }
    // Each body's box half extents (for drawing; the jacks' too).
    const Math::vec3& halfExtents(u32 index) const
    {
        return mHalfExtents[index];
    }
    u32 bodyId(u32 index) const
    {
        return mIds[index];
    }
    HingeJoint& slewJoint()
    {
        return *mSlew;
    }
    HingeJoint& luffJoint()
    {
        return *mLuff;
    }
    SliderJoint& telescope(u32 index)
    {
        return *mTelescopes[index];
    }
    SliderJoint& outriggerBeam(u32 index)
    {
        return *mBeamSliders[index];
    }
    SliderJoint& outriggerJack(u32 index)
    {
        return *mJackSliders[index];
    }
    HydraulicCylinder& luffCylinder()
    {
        return *mLuffCylinder;
    }
    HydraulicMotor& slewMotor()
    {
        return *mSlewMotor;
    }
    HydraulicSystem& hydraulics()
    {
        return *mHydraulics;
    }
    Rope& rope()
    {
        return *mRope;
    }
    Winch& winch()
    {
        return mWinch;
    }
    Hook& hook()
    {
        return *mHook;
    }
    const MobileCraneParameters& parameters() const
    {
        return mParameters;
    }

    enum BodyIndex : u32
    {
        Chassis,
        Beam0,
        Jack0 = Beam0 + 4,
        Superstructure = Jack0 + 4,
        BoomBase,
        HookBlock = BoomBase + 1 + MobileCraneParameters::SectionCount,
        BodyCount
    };

private:
    void addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation, const Math::vec3& halfExtents,
                 f32 mass, const CollisionFilter& filter, bool sphere = false);
    Math::vec3 carrierWorld(const Math::vec3& local) const;
    Math::vec3 superWorld(const Math::vec3& local) const;
    Math::vec3 boomWorld(f32 along, f32 up) const;

    PhysicsWorld* mWorld;
    MobileCraneParameters mParameters;
    MobileCraneControls mControls;
    Math::vec3 mPosition{0.0f};
    Math::quat mOrientation;
    Math::quat mBoomOrientation;
    Math::quat mSuperOrientation;
    Math::vec3 mSlewWorld{0.0f};
    Math::vec3 mFootWorld{0.0f};

    RigidBody mBodies[BodyCount];
    CollisionShape* mShapes[BodyCount] = {};
    Math::vec3 mHalfExtents[BodyCount];
    u32 mIds[BodyCount] = {};

    SliderJoint* mBeamSliders[4] = {};
    SliderJoint* mJackSliders[4] = {};
    HingeJoint* mSlew = nullptr;
    HingeJoint* mLuff = nullptr;
    SliderJoint* mTelescopes[MobileCraneParameters::SectionCount] = {};
    HydraulicCylinder* mLuffCylinder = nullptr;
    HydraulicMotor* mSlewMotor = nullptr;
    HydraulicSystem* mHydraulics = nullptr;
    Winch mWinch;
    Rope* mRope = nullptr;
    Hook* mHook = nullptr;
    u32 mLuffSection = 0;
    u32 mSlewSection = 0;

    // Pins on both bodies for the separation readout: slewing ring, boom foot, telescopic sections' lines.
    Math::vec3 mSlewLocal[2];
    Math::vec3 mFootLocal[2];
    Math::vec3 mHeadLocal{0.0f};
    Math::vec3 mSlewTopLocal{0.0f};

    LoadMomentIndicator mLmi;
    bool mAntiSway = false;
    InputShaper mShaper;
    f32 mTelescopeTarget = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_MOBILECRANE_H
