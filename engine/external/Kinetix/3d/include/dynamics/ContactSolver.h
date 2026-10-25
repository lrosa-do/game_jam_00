#ifndef KINETIX_PHYSICS_DYNAMICS_CONTACTSOLVER_H
#define KINETIX_PHYSICS_DYNAMICS_CONTACTSOLVER_H

#include "collision/Narrowphase.h"
#include "dynamics/ArticulationSolver.h"

namespace kinetix::Physics
{

class RigidBody;
class Joint;

struct Contact
{
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    // The owner's own body numbering; the solver never reads these. Only the
    // pairs the narrowphase confirms become contacts, so they cannot be a
    // parallel array.
    u32 idA = 0;
    u32 idB = 0;
    ContactManifold manifold;
    // Combined from the two materials by the caller.
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
    // Set by the solver for a contact the articulation solver took this step.
    bool articulated = false;
};

struct ContactSolverSettings
{
    // More velocity iterations make a stack settle rather than sag.
    u32 velocityIterations = 8;
    // Position correction needs fewer: it fixes what the velocity pass could not.
    u32 positionIterations = 3;
    // Fraction of the remaining overlap removed per position iteration.
    f32 baumgarte = 0.2f;
    // Overlap left alone, so bodies do not chatter around exactly zero.
    f32 slop = 0.005f;
    // Overlap beyond this is corrected as if it were this deep, so deep
    // penetrations are pushed out over a few steps instead of flung.
    f32 maxPenetration = 0.2f;
    // Approach speed below which a contact is treated as resting.
    f32 restitutionThreshold = 1.0f;
};

 
class ContactSolver
{
public:
    void setSettings(const ContactSolverSettings& settings)
    {
        mSettings = settings;
    }
    const ContactSolverSettings& settings() const
    {
        return mSettings;
    }

 
    // Velocity half of a step: restitution, warm start and the velocity
    // iterations of contacts and joints. The caller integrates positions next,
    // then calls solvePositions(), so the overlap a step's own motion creates
    // is removed in that same step.
    //
    // A contact or joint whose dynamic bodies are all asleep is skipped by both
    // halves. Joints are still set up, since that is where a motor wakes its bodies.
    //
    // Articulated joints (Joint::setArticulated) are set up and warm started
    // here, then solved by the ArticulationSolver after the contacts and
    // iterative joints in every iteration of both halves.
    void solve(Contact* contacts, u32 count, Joint* const* joints, u32 jointCount, f32 duration);
    void solve(Contact* contacts, u32 count, f32 duration)
    {
        solve(contacts, count, nullptr, 0, duration);
    }
    void solvePositions(Contact* contacts, u32 count, Joint* const* joints, u32 jointCount);
    void solvePositions(Contact* contacts, u32 count)
    {
        solvePositions(contacts, count, nullptr, 0);
    }

    const ArticulationSolver& articulation() const
    {
        return mArticulation;
    }

private:
    void warmStart(Contact* contacts, u32 count);
    void solveVelocity(Contact* contacts, u32 count);
    void solvePosition(Contact* contacts, u32 count);

    ContactSolverSettings mSettings;
    ArticulationSolver mArticulation;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_CONTACTSOLVER_H
