#ifndef KINETIX_PHYSICS_ACTUATORS_ROPE_H
#define KINETIX_PHYSICS_ACTUATORS_ROPE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Joint.h"

namespace kinetix::Physics
{

class RigidBody;

struct WinchParameters
{
    // Drum radius to the rope's centre at the working layer, m.
    f32 drumRadius = 0.25f;
    // The most the motor pulls on the line (one part), N, at any speed up to
    // maxLineSpeed - a hydraulic winch motor behind its relief valve.
    f32 maxLinePull = 20.0e3f;
    // Line speed at full command, m/s.
    f32 maxLineSpeed = 0.5f;
    // What the brake holds, as a line pull, N. <= 0: 1.5 x maxLinePull.
    f32 brakeLinePull = 0.0f;
    // Drum and motor inertia seen at the drum, kg*m^2. <= 0: the inertia of
    // a 50 kg drum at its radius.
    f32 drumInertia = 0.0f;
    // Engages by itself whenever the command is zero.
    bool autoBrake = true;
};

// A winch drum with its motor and brake, at the first end of a Rope
// (Rope::setWinch). Its line speed is a degree of freedom solved with the
// rope: the motor drives it to the commanded speed with at most maxLinePull,
// the brake holds with at most brakeLinePull, and past maxLinePull it stalls
// (or runs back).
class Winch
{
public:
    explicit Winch(const WinchParameters& parameters = WinchParameters());

    // -1..1 of maxLineSpeed, + hoists (pays line in). Clamped; non-finite
    // values are ignored.
    void setCommand(f32 command);
    f32 command() const
    {
        return mCommand;
    }
    // A brake applied by hand, on top of the automatic one.
    void setBrake(bool engaged);
    // Declutched: the motor lets go and the automatic brake is off, so the
    // drum runs free unless braked by hand.
    void setFreeSpool(bool freeSpool);
    bool freeSpool() const
    {
        return mFreeSpool;
    }
    bool braking() const;

    const WinchParameters& parameters() const
    {
        return mParameters;
    }

    // Line speed at the drum, m/s, positive paying out; drum turn rate and
    // angle (rad, positive paying out).
    f32 lineSpeed() const
    {
        return mLineSpeed;
    }
    f32 drumAngularVelocity() const
    {
        return mLineSpeed / mParameters.drumRadius;
    }
    f32 drumAngle() const
    {
        return mDrumAngle;
    }
    // Last step: the motor's torque on the drum (N*m, positive hoisting) and
    // the brake's (N*m, the magnitude it held).
    f32 motorTorque() const
    {
        return mMotorTorque;
    }
    f32 brakeTorque() const
    {
        return mBrakeTorque;
    }

private:
    friend class Rope;

    WinchParameters mParameters;
    f32 mCommand = 0.0f;
    bool mManualBrake = false;
    bool mFreeSpool = false;
    f32 mLineSpeed = 0.0f;
    f32 mDrumAngle = 0.0f;
    f32 mMotorTorque = 0.0f;
    f32 mBrakeTorque = 0.0f;
    f32 mEquivalentMass = 50.0f;
};

// A rope (or wire rope, chain) between a point on one body and a point on
// another, over any number of sheaves: a one-sided distance constraint on the
// total path length. Slack it does nothing; taut it holds the length,
// optionally elastic.
//
//  - Sheaves are points fixed on bodies, in order from A to B. With a radius
//    the rope leaves along the tangents and wraps r * phi (the wrap plane is
//    the one through the sheave and its two neighbours); with efficiency
//    eta < 1 the tension rises by 1/eta across it in the running direction:
//    hoisting through k sheaves the drum pulls W / eta^k, lowering only
//    W eta^k.
//  - `falls` is the parts of line between the last sheave (or A) and B: a
//    hook block reeved with N falls hangs on N parts and moves 1/N of the
//    line paid in.
//  - The rest length changes with a winch at A (setWinch) or by hand
//    (setRestLength).
//
// bodyA()/bodyB() are the first and last points' bodies; dynamic sheave
// bodies in between must be held to one of them by joints of their own (a
// boom tip on its boom), which puts them in the same sleep island.
class Rope final : public Joint
{
public:
    static constexpr u32 MaxPoints = 8;

    // A rest length <= 0 takes the path's length - sheaves and falls
    // included - when the rope is first solved (or restLength() is asked).
    Rope(RigidBody& a, const Math::vec3& worldPointA, RigidBody& b, const Math::vec3& worldPointB,
         f32 restLength = 0.0f);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    // Articulation (Joint::setArticulated): the rope's row over all its bodies
    // and the winch's drum are solved with the articulation's locks, so a
    // load hanging from a boom is carried by the whole boom in one iteration.
    // A route over more distinct bodies than Joint::MaxArticulationBodies
    // stays iterative.
    bool supportsArticulation() const override;
    u32 articulationBodies(RigidBody** bodies) override;
    u32 articulationWideMotors(ArticulationRow* rows) override;
    void setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass) override;
    f32 solveArticulationMotor(u32 id, f32 velocity) override;
    u32 articulationWidePositionRows(ArticulationRow* rows, f32* errors) override;

    // Adds a sheave after the ones already there: its centre (world), its
    // radius to the rope's centre (m, 0 = a point) and its efficiency (the
    // tension ratio across it, 0 < eta <= 1). False when the route is
    // full. Does not change the rest length.
    bool addSheave(RigidBody& body, const Math::vec3& worldPoint, f32 radius = 0.0f, f32 efficiency = 1.0f);
    u32 pointCount() const
    {
        return mCount;
    }
    Math::vec3 pointWorld(u32 index) const;
    RigidBody* pointBody(u32 index) const;

    void setFalls(u32 falls);
    u32 falls() const
    {
        return mFalls;
    }

    // Elasticity: EA, the axial stiffness of the rope's section (N; a 20 mm
    // steel wire rope is ~25e6), and the damping ratio of the stretch. Zero
    // EA is an inextensible rope.
    void setElasticity(f32 axialStiffness, f32 dampingRatio = 0.5f);

    // Length of rope in the path, falls counted, m.
    void setRestLength(f32 length);
    f32 restLength() const
    {
        return mRestLength >= 0.0f ? mRestLength : length();
    }
    // The winch will not pay in below `minLength` (two-blocking) or out past
    // `maxLength` (an empty drum).
    void setLengthLimits(f32 minLength, f32 maxLength);
    f32 length() const;
    // length() - restLength(): negative slack, positive stretched.
    f32 stretch() const
    {
        return length() - restLength();
    }

    // Puts a winch at A. The winch is not owned; null removes it.
    void setWinch(Winch* winch);
    Winch* winch() const
    {
        return mWinch;
    }

    // Sensors, last step: the tension in the line (one part, N), whether the
    // rope carried any, and the load on B (tension * falls, N).
    // The line tension at A (the drum's line pull) - with sheave friction
    // the segments further along differ (segmentTension).
    f32 tension() const;
    bool taut() const
    {
        return tension() > 0.0f;
    }
    f32 segmentTension(u32 index) const;
    f32 hookLoad() const
    {
        return segmentTension(mCount - 2) * static_cast<f32>(mFalls);
    }
    // The pull on B at its point (Joint::reactionForce conventions).
    Math::vec3 reactionForce() const override;

private:
    struct Point
    {
        RigidBody* body = nullptr;
        Math::vec3 local{0.0f};
        f32 radius = 0.0f;
        f32 efficiency = 1.0f;
    };
    // The route at the bodies' current pose: each segment's direction and
    // straight length between the tangent points of the sheaves.
    struct Path
    {
        Math::vec3 direction[MaxPoints];
        f32 segment[MaxPoints] = {};
        f32 length = 0.0f;
    };
    struct BodyPart
    {
        RigidBody* body = nullptr;
        // The length's gradient (what the row measures) and the tension's
        // (what an impulse applies): equal unless sheaves have friction.
        Math::vec3 linear{0.0f};
        Math::vec3 angular{0.0f};
        Math::vec3 forceLinear{0.0f};
        Math::vec3 forceAngular{0.0f};
    };

    f32 pathLength(Math::vec3* directions) const;
    void computePath(Path& path) const;
    void updateFriction(const Path& path);
    // With `friction`, the tension factors are re-read from how the rope
    // runs now (setup); the position pass keeps the step's.
    void calculateJacobian(bool friction);
    f32 bodyVelocity() const;
    void applyBodyImpulse(f32 impulse);
    void wakeRoute();
    void advanceDrum();
    f32 solveRow(f32 rate, f32 bodies);

    Point mPoints[MaxPoints];
    u32 mCount = 0;
    u32 mFalls = 1;
    f32 mRestLength = 0.0f;
    f32 mMinLength = 0.0f;
    f32 mMaxLength = 1.0e9f;
    f32 mAxialStiffness = 0.0f;
    f32 mDampingRatio = 0.5f;
    Winch* mWinch = nullptr;

    BodyPart mParts[MaxPoints];
    u32 mPartCount = 0;
    Math::vec3 mLastDirection{0.0f, -1.0f, 0.0f};
    f32 mLength = 0.0f;
    f32 mInverseMass = 0.0f;
    // J M^-1 J^T with the length's own gradient, for the position pass.
    f32 mGeometricInverseMass = 0.0f;
    f32 mTensionFactor[MaxPoints] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    bool mHasFriction = false;
    // The last plane each sheave wrapped in (world), for a route that runs
    // straight back on itself and has no plane of its own.
    mutable Math::vec3 mPlane[MaxPoints];
    f32 mBias = 0.0f;
    f32 mSoftness = 0.0f;
    bool mRigid = true;
    f32 mTotalImpulse = 0.0f;
    f32 mArticulatedInverseMass = 0.0f;

    bool mDrumActive = false;
    f32 mDrumTarget = 0.0f;
    f32 mDrumMinImpulse = 0.0f;
    f32 mDrumMaxImpulse = 0.0f;
    f32 mTotalDrumImpulse = 0.0f;
    bool mDrumBraking = false;
    bool mDrumAdvanced = true;
    f32 mDuration = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ACTUATORS_ROPE_H
