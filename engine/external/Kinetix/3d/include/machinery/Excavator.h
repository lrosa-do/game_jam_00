#ifndef KINETIX_PHYSICS_MACHINERY_EXCAVATOR_H
#define KINETIX_PHYSICS_MACHINERY_EXCAVATOR_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/HydraulicCylinder.h"
#include "actuators/HydraulicMotor.h"
#include "collision/CollisionFilter.h"
#include "collision/CollisionShape.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/RigidBody.h"
#include "machinery/HydraulicSystem.h"
#include "machinery/SoilCutting.h"
#include "machinery/TrackedUndercarriage.h"

namespace kinetix::Physics
{

class PhysicsWorld;

// A point of the arm's plane: forward along the upper structure (z), up (y), from the slewing ring's centre, m.
struct ArmPoint
{
    f32 z = 0.0f;
    f32 y = 0.0f;
};

// The arm's geometry in the upper structure's frame at the build pose (every joint angle zero).
// The arm moves in the plane x = 0. The two boom cylinders at +-boomCylinderSpread push alike and
// are simulated as one ganged cylinder on the centre line (HydraulicCylinderParameters::count = 2).
//
// Pins: boomFoot on the upper; stickPin (boom tip); bucketPin (stick tip). Boom cylinders
// boomBarrel (upper) to boomRod (boom); stick cylinder stickBarrel (boom) to stickRod (above stickPin).
// Bucket four-bar on the stick: bucket cylinder from bucketBarrel to linkPin on the rocker, the
// rocker hinged on the stick at rockerPin, the H-link (rigid distance) from linkPin to
// bucketLinkPin on the bucket, the bucket hinged at bucketPin. The bucket: a back plate from
// bucketPin along bucketBack, then the lip carrying the teeth.
//
// Defaults: the boom cylinders swing the boom from -60 to +25 deg about its 40 deg build
// elevation over 1.4 m, the stick from 55 deg out to 60 deg in over 1.5 m, clear of dead centres.
// The Grashof four-bar (ground link 0.35 m, rocker 0.6, H-link 0.55, bucket arm 0.4) turns the
// bucket 168 deg over the 1.0 m bucket stroke, with H-link and cylinder 33 deg or more from a dead centre.
struct ExcavatorGeometry
{
    ArmPoint boomFoot{0.6f, 1.0f};
    ArmPoint stickPin{4.966f, 4.664f};
    ArmPoint bucketPin{5.717f, 1.863f};
    ArmPoint boomBarrel{1.2f, 0.15f};
    ArmPoint boomRod{2.720f, 2.518f};
    ArmPoint stickBarrel{2.577f, 3.311f};
    ArmPoint stickRod{4.540f, 5.482f};
    ArmPoint bucketBarrel{5.482f, 4.285f};
    ArmPoint rockerPin{5.626f, 2.201f};
    ArmPoint linkPin{6.224f, 2.253f};
    ArmPoint bucketLinkPin{6.091f, 1.720f};
    // Back plate leaves the pin at this angle (rad, arm plane from +z towards +y) for this length;
    // the lip turns square to it towards the machine; width across, m.
    f32 bucketBackAngle = -1.745f;
    f32 bucketBackLength = 1.1f;
    f32 bucketLipLength = 0.7f;
    f32 bucketWidth = 1.0f;
    f32 boomCylinderSpread = 0.45f;
};

// The arm's planar kinematics in closed form: cylinder length <-> joint angle, and the bucket's
// four-bar. Angles are the hinges' (zero at the build pose; positive raises the boom, moves the
// stick out, opens the bucket; counter-clockwise with +z right, +y up).
class ExcavatorKinematics
{
public:
    explicit ExcavatorKinematics(const ExcavatorGeometry& geometry);

    f32 boomCylinderLength(f32 boomAngle) const;
    f32 stickCylinderLength(f32 stickAngle) const;
    // The inverse, within +-pi of the build pose.
    f32 boomAngle(f32 cylinderLength) const;
    f32 stickAngle(f32 cylinderLength) const;
    // Four-bar from the bucket cylinder's length: the rocker's angle about its pin and the
    // bucket's about the bucket pin (relative to the stick). False if the linkage cannot close.
    bool bucketLinkage(f32 cylinderLength, f32* rockerAngle, f32* bucketAngle) const;
    // Bucket cylinder length for a bucket angle (relative to the stick), searched in
    // [minLength, maxLength]; false if out of reach.
    bool bucketCylinderLength(f32 bucketAngle, f32 minLength, f32 maxLength, f32* length) const;

    ArmPoint toothTip(f32 boomAngle, f32 stickAngle, f32 bucketAngle) const;
    ArmPoint bucketPinAt(f32 boomAngle, f32 stickAngle) const;
    ArmPoint restToothTip() const;
    f32 restAttitude() const;
    // Two-link inverse kinematics: boom and stick angles putting the bucket pin at `pin` (elbow up).
    bool armAngles(const ArmPoint& pin, f32* boomAngle, f32* stickAngle) const;

    const ExcavatorGeometry& geometry() const
    {
        return mGeometry;
    }

private:
    ExcavatorGeometry mGeometry;
    f32 mBoomLength = 0.0f;
    f32 mStickLength = 0.0f;
    f32 mBoomRestAngle = 0.0f;
    f32 mStickRestAngle = 0.0f;
    f32 mRockerLength = 0.0f;
    f32 mLinkLength = 0.0f;
    f32 mBucketLinkRadius = 0.0f;
    f32 mRockerRestAngle = 0.0f;
    f32 mBucketLinkRestAngle = 0.0f;
    f32 mRockerSide = 1.0f;
    f32 mBucketSide = 1.0f;
};

struct ExcavatorParameters
{
    ExcavatorGeometry geometry;
    TrackParameters tracks;
    // Masses, kg.
    f32 undercarriageMass = 7500.0f;
    f32 upperMass = 9000.0f;
    f32 boomMass = 1500.0f;
    f32 stickMass = 700.0f;
    f32 bucketMass = 650.0f;
    f32 rockerMass = 50.0f;
    // Undercarriage frame half extents, m; its centre of mass height over the slewing ring's plane.
    Math::vec3 undercarriageHalfExtents{1.3f, 0.35f, 2.0f};
    f32 ringHeight = 0.35f;
    // Upper structure box (half extents) and its centre of mass from the ring's centre (rearward: counterweight).
    Math::vec3 upperHalfExtents{1.35f, 0.55f, 1.7f};
    Math::vec3 upperCentre{0.0f, 0.55f, -0.9f};

    EngineParameters engine;
    HydraulicPumpParameters pump;
    HydraulicCylinderParameters boomCylinder;
    HydraulicCylinderParameters stickCylinder;
    HydraulicCylinderParameters bucketCylinder;
    HydraulicMotorParameters swingMotor;
    HydraulicMotorParameters travelMotor;
    ValveSectionParameters boomSection;
    ValveSectionParameters stickSection;
    ValveSectionParameters bucketSection;
    ValveSectionParameters swingSection;
    ValveSectionParameters travelSection;

    // Bodies collide with bodyFilter (not each other); the bucket skips diggableGroup (the soil
    // bed's group) so it can enter the soil. The tracks' ground probes skip the machine's own group.
    CollisionFilter bodyFilter{1u << 6, ~(1u << 6)};
    u32 diggableGroup = 1u << 7;

    // 20 t class: 110 kW, two 120 cc load-sensing pumps with a 90 kW power limit, 5.7 m boom,
    // 2.9 m stick, 1 m^3 bucket, slew 11 rpm, travel 5 km/h.
    static ExcavatorParameters crawler20t();
};

// The operator's levers, -1..1 each.
struct ExcavatorControls
{
    // Travel: + forward, each track.
    f32 leftTrack = 0.0f;
    f32 rightTrack = 0.0f;
    // + slews to the left (counter-clockwise from above).
    f32 swing = 0.0f;
    // + raises the boom, + brings the stick in (towards the machine), +
    // curls the bucket in.
    f32 boom = 0.0f;
    f32 stick = 0.0f;
    f32 bucket = 0.0f;
    // Engine speed dial, rpm (0: the engine's rated speed).
    f32 engineRpm = 0.0f;
};

// A crawler excavator built into a world. Every joint of the arm and the slewing ring is
// articulated (Joint::setArticulated): the arm and its linkage are solved as one block.
// The machine owns its bodies, shapes and joints; the world does not.
//
// Call update(step) before every world step.
class Excavator
{
public:
    Excavator(PhysicsWorld& world, const ExcavatorParameters& parameters, const Math::vec3& position,
              const Math::quat& orientation = Math::quat());
    ~Excavator();
    Excavator(const Excavator&) = delete;
    Excavator& operator=(const Excavator&) = delete;

    void setControls(const ExcavatorControls& controls);
    const ExcavatorControls& controls() const
    {
        return mControls;
    }
    // Operator assist: drives boom, stick and bucket to put the teeth at `toothTip` (arm plane, m,
    // from the ring's centre) with the bucket at `attitude` (rad, angle of pin-to-teeth), closing a
    // loop on the cylinder lengths from the kinematics. Its spools replace the boom, stick and
    // bucket levers until cleared.
    void setToothTarget(const ArmPoint& toothTip, f32 attitude);
    void clearToothTarget();
    bool toothTargetActive() const
    {
        return mAssist;
    }
    // Same for the swing to an angle (rad, the slewing hinge's).
    void setSwingTarget(f32 angle);
    void clearSwingTarget();
    // The bucket's soil (teeth cutting tool) and the soil the tracks stand on.
    void setSoilBed(const SoilBed& bed);
    void setGroundSoil(const SoilParameters& soil);

    void update(f32 step);

    RigidBody& undercarriage()
    {
        return mBodies[Undercarriage];
    }
    RigidBody& upper()
    {
        return mBodies[Upper];
    }
    RigidBody& boom()
    {
        return mBodies[Boom];
    }
    RigidBody& stick()
    {
        return mBodies[Stick];
    }
    RigidBody& bucket()
    {
        return mBodies[BucketBack];
    }
    RigidBody& bucketLip()
    {
        return mBodies[BucketLip];
    }
    RigidBody& rocker()
    {
        return mBodies[Rocker];
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
    static constexpr u32 BodyCount = 7;
    enum BodyIndex : u32
    {
        Undercarriage,
        Upper,
        Boom,
        Stick,
        BucketBack,
        BucketLip,
        Rocker
    };

    HingeJoint& swingJoint()
    {
        return *mSwing;
    }
    HingeJoint& boomJoint()
    {
        return *mBoomHinge;
    }
    HingeJoint& stickJoint()
    {
        return *mStickHinge;
    }
    HingeJoint& bucketJoint()
    {
        return *mBucketHinge;
    }
    HingeJoint& rockerJoint()
    {
        return *mRockerHinge;
    }
    DistanceJoint& linkJoint()
    {
        return *mLink;
    }
    HydraulicCylinder& boomCylinder()
    {
        return *mBoomCylinder;
    }
    HydraulicCylinder& stickCylinder()
    {
        return *mStickCylinder;
    }
    HydraulicCylinder& bucketCylinder()
    {
        return *mBucketCylinder;
    }
    HydraulicMotor& swingMotor()
    {
        return *mSwingMotor;
    }
    HydraulicSystem& hydraulics()
    {
        return *mHydraulics;
    }
    TrackedUndercarriage& tracks()
    {
        return *mTracks;
    }
    SoilCuttingTool& teeth()
    {
        return *mTeeth;
    }
    const ExcavatorKinematics& kinematics() const
    {
        return mKinematics;
    }
    const ExcavatorParameters& parameters() const
    {
        return mParameters;
    }

    // World position of the teeth (lip's edge, mid-width), the ring's centre and the bucket pin.
    Math::vec3 toothTipWorld() const;
    Math::vec3 ringCentreWorld() const;
    Math::vec3 bucketPinWorld() const;
    ArmPoint toothTip() const;
    f32 attitude() const;
    // Worst pin separation over the articulated joints, m.
    f32 worstJointSeparation() const;
    bool finite() const;

    enum Section : u32
    {
        BoomSection,
        BucketSection,
        LeftTravelSection,
        StickSection,
        SwingSection,
        RightTravelSection,
        SectionCount
    };
    u32 section(Section which) const
    {
        return mSections[which];
    }

private:
    Math::vec3 armWorld(const ArmPoint& point, f32 x = 0.0f) const;
    void buildBodies(const Math::vec3& position, const Math::quat& orientation);
    void buildJoints();
    void buildHydraulics();
    void addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation, const Math::vec3& halfExtents,
                 f32 mass, const CollisionFilter& filter);
    Math::quat memberOrientation(const ArmPoint& from, const ArmPoint& to) const;
    void runAssist();

    PhysicsWorld* mWorld;
    ExcavatorParameters mParameters;
    ExcavatorKinematics mKinematics;
    ExcavatorControls mControls;

    RigidBody mBodies[BodyCount];
    BoxShape* mShapes[BodyCount] = {};
    u32 mIds[BodyCount] = {};
    Math::vec3 mRingCentre{0.0f};
    Math::quat mBuildOrientation;

    HingeJoint* mSwing = nullptr;
    HingeJoint* mBoomHinge = nullptr;
    HingeJoint* mStickHinge = nullptr;
    HingeJoint* mBucketHinge = nullptr;
    HingeJoint* mRockerHinge = nullptr;
    DistanceJoint* mLink = nullptr;
    FixedJoint* mBucketWeld = nullptr;
    HydraulicCylinder* mBoomCylinder = nullptr;
    HydraulicCylinder* mStickCylinder = nullptr;
    HydraulicCylinder* mBucketCylinder = nullptr;
    HydraulicMotor* mSwingMotor = nullptr;
    HydraulicMotor* mTravelMotors[2] = {};
    HydraulicSystem* mHydraulics = nullptr;
    TrackedUndercarriage* mTracks = nullptr;
    SoilCuttingTool* mTeeth = nullptr;
    u32 mSections[SectionCount] = {};

    // Pins of the arm's hinges and the bucket's weld, on both bodies: ring, boom foot, stick pin,
    // bucket pin, rocker pin, weld.
    static constexpr u32 PinCount = 6;
    struct Pin
    {
        u32 bodyA = 0;
        u32 bodyB = 0;
        Math::vec3 localA{0.0f};
        Math::vec3 localB{0.0f};
    };
    Pin mPins[PinCount];

    bool mAssist = false;
    ArmPoint mToothTarget;
    f32 mAttitudeTarget = 0.0f;
    bool mSwingAssist = false;
    f32 mSwingTarget = 0.0f;
    f32 mAssistBoom = 0.0f;
    f32 mAssistStick = 0.0f;
    f32 mAssistBucket = 0.0f;
    f32 mAssistSwing = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_EXCAVATOR_H
