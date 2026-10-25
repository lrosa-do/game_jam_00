#ifndef KINETIX_PHYSICS_DYNAMICS_JOINT_H
#define KINETIX_PHYSICS_DYNAMICS_JOINT_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class RigidBody;

// One constraint row's Jacobian, split per body: the row's velocity is
// linearB.vB + angularB.wB + linearA.vA + angularA.wA, and an impulse
// lambda along it changes each body's momentum by lambda times its part.
struct JointRow
{
    Math::vec3 linearA{0.0f};
    Math::vec3 angularA{0.0f};
    Math::vec3 linearB{0.0f};
    Math::vec3 angularB{0.0f};
};

// A bounded row (motor, servo, limit, end stop) the articulation solver drives
// for the joint; `id` is the joint's own name for it, handed back in the calls below.
struct ArticulationMotor
{
    JointRow row;
    u32 id = 0;
    // Gains scaled by the row's inertia (a frequency servo): only then is the
    // articulation's joint-space inertia worked out for it.
    bool wantsGain = false;
};

// A row over up to MaxParts bodies (a rope over sheaves). Per body: the
// velocity Jacobian (linear, angular about the centre of mass) and, when an
// impulse acts differently from how the row is measured (rope tension over
// frictional sheaves), the Jacobian the impulse is applied along.
struct ArticulationRow
{
    static constexpr u32 MaxParts = 4;
    u32 parts = 0;
    RigidBody* body[MaxParts] = {};
    Math::vec3 linear[MaxParts];
    Math::vec3 angular[MaxParts];
    bool asymmetric = false;
    Math::vec3 forceLinear[MaxParts];
    Math::vec3 forceAngular[MaxParts];
    u32 id = 0;
};

class Joint
{
public:
    static constexpr u32 MaxArticulationRows = 6;
    static constexpr u32 MaxArticulationBodies = ArticulationRow::MaxParts;

    virtual ~Joint() = default;

    virtual RigidBody* bodyA() const = 0;
    virtual RigidBody* bodyB() const = 0;
    // A single-body joint reports the same body at both ends (a drag spring to
    // a world target); the world's degenerate-pair rejection must let it through.
    virtual bool singleBody() const
    {
        return false;
    }
    virtual void setup(f32 duration) = 0;
    virtual void warmStart() = 0;
    virtual void solveVelocity() = 0;
    virtual void solvePosition(f32 baumgarte) = 0;

    // Total force and torque this joint applied to body B over the last solved
    // step (accumulated impulses / step), world space, N and N*m. The force acts
    // at B's anchor, the torque is about that point; A gets the opposite. A
    // hanging mass reads +m*g upwards. Zero before the first solve; a sleeping
    // joint keeps its last reading.
    virtual Math::vec3 reactionForce() const
    {
        return Math::vec3(0.0f);
    }
    virtual Math::vec3 reactionTorque() const
    {
        return Math::vec3(0.0f);
    }

    void setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }
    bool enabled() const
    {
        return mEnabled;
    }

    // Opts into the articulation solver (ArticulationSolver.h): all articulated
    // joints connected through dynamic bodies are solved as one block (exact
    // lock rows, drives and limits with the whole articulation's inertia). For
    // arms, booms and linkages, which the iterative solver lets drift. Off by
    // default; a joint with supportsArticulation() false stays iterative.
    void setArticulated(bool articulated)
    {
        mArticulated = articulated;
    }
    bool articulated() const
    {
        return mArticulated;
    }
    virtual bool supportsArticulation() const
    {
        return false;
    }
    // True if the articulation solver took this joint this step: the world's
    // solver then calls only setup() and warmStart() on it.
    bool solvedByArticulation() const
    {
        return mInArticulation;
    }

    // The interface the articulation solver works through: world space, at the
    // bodies' current pose, only after setup().
    //
    // Equality rows (locks) and their position errors C, signed so that dC/dt
    // is the row's velocity. At most MaxArticulationRows; `errors` may be null.
    virtual u32 articulationLocks(JointRow* rows, f32* errors)
    {
        (void)rows;
        (void)errors;
        return 0;
    }
    // The velocity pass's impulses on those rows, in articulationLocks() order,
    // for the warm start and sensors.
    virtual void addArticulationLockImpulses(const f32* impulses)
    {
        (void)impulses;
    }
    virtual u32 articulationMotors(ArticulationMotor* motors)
    {
        (void)motors;
        return 0;
    }
    // Inverse effective mass a motor row is solved with (all locks holding), and
    // the one servo gains are scaled by (all other drives holding too: the
    // inertia the axis drives, outboard links included).
    virtual void setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
    {
        (void)id;
        (void)inverseMass;
        (void)gainInverseMass;
    }
    // One iteration at row velocity `velocity`: returns the impulse to add (the
    // joint accumulates and bounds it; the solver applies it).
    virtual f32 solveArticulationMotor(u32 id, f32 velocity)
    {
        (void)id;
        (void)velocity;
        return 0.0f;
    }
    // Position rows besides the locks, projected with them: a rigid servo's
    // remaining error, a limit or stop the joint is beyond.
    virtual u32 articulationPositionRows(JointRow* rows, f32* errors)
    {
        (void)rows;
        (void)errors;
        return 0;
    }
    // For a joint over more than two bodies (rope over sheaves): the bodies it
    // joins and its bounded and position rows over them. May be combined with
    // the two-body calls (ids shared).
    virtual u32 articulationBodies(RigidBody** bodies)
    {
        bodies[0] = bodyA();
        bodies[1] = bodyB();
        return 2;
    }
    virtual u32 articulationWideMotors(ArticulationRow* rows)
    {
        (void)rows;
        return 0;
    }
    virtual u32 articulationWidePositionRows(ArticulationRow* rows, f32* errors)
    {
        (void)rows;
        (void)errors;
        return 0;
    }

protected:
    // A motor/servo still working must call this from setup(): a settled body
    // is asleep and skipped, so a motor on it would otherwise never move anything.
    void wakeBodies() const
    {
        if (RigidBody* a = bodyA())
            wakeBody(a);
        if (RigidBody* b = bodyB())
            wakeBody(b);
    }

private:
    friend class ArticulationSolver;

    static void wakeBody(RigidBody* body);

    bool mEnabled = true;
    bool mArticulated = false;
    bool mInArticulation = false;
};

}

#endif
