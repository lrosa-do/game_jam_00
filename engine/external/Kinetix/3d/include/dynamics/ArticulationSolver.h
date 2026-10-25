#ifndef KINETIX_PHYSICS_DYNAMICS_ARTICULATIONSOLVER_H
#define KINETIX_PHYSICS_DYNAMICS_ARTICULATIONSOLVER_H

#include <kinetix/core/Containers.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Joint.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RigidBody;
struct Contact;

// Direct solver for articulated joints (Joint::setArticulated): chains and
// loops of heavy bodies (excavator arm, crane boom, linkage) where the
// iterative Gauss-Seidel solver is not accurate enough. Owned by ContactSolver.
//
// Per group of articulated joints connected through dynamic bodies:
//  - Velocity: each iteration, after contacts and iterative joints, the lock
//    rows are solved exactly: one solve of K = J M^-1 J^T, factorised once a
//    step (LDL^T within each row's envelope, joints ordered from the leaves
//    towards the hub to limit fill-in). An iteration where nothing touched the
//    group's bodies since the last one is skipped. The bounded rows (drives,
//    limits, stops, ropes, contacts) are then solved by projected Gauss-Seidel
//    in the motor space the locks leave: each impulse carries the lock impulses
//    -K^-1 c that keep the locks satisfied, giving the Schur complement
//    A = J_m M^-1 F_m^T - c^T K^-1 c (built once a step).
//  - Position: on the first position pass the lock errors (plus violated
//    limits, a stretched rope, contact overlap) are removed by one Newton step
//    of the whole group (K dlambda = -C as a position impulse); on the last
//    pass again, reusing the factor with the new pose's errors (quasi-Newton)
//    while the rows are the same, else refactorising.
//  - Gains: frequency servos are scaled by the inertia their axis moves with
//    all other drives holding (diagonal of the joint-space mass matrix), not
//    just their own pair of bodies.
//
// Redundant rows (a closed loop of hinges) are found by the factorisation and
// dropped. A group of more than MaxGroupRows lock rows falls back to the
// iterative solver.
class ArticulationSolver
{
public:
    static constexpr u32 MaxGroupRows = 192;

    // After every joint's setup() and the contacts' preparation. Also takes the
    // contacts between a group's bodies and static or kinematic ground (or
    // within one group) as bounded rows (normal >= 0, friction within mu times
    // the normal) and marks them (Contact::articulated); their position
    // correction is then the articulation's.
    void prepare(Joint* const* joints, u32 count, Contact* contacts, u32 contactCount, f32 duration,
                 f32 baumgarte, f32 slop, f32 maxPenetration);
    void prepare(Joint* const* joints, u32 count, f32 duration)
    {
        prepare(joints, count, nullptr, 0, duration, 0.2f, 0.005f, 0.2f);
    }
    void solveVelocity();
    // After the last velocity iteration: lock impulses go back into the joints
    // for their warm start and sensors.
    void finishVelocity();
    // Projection on the first pass of a step (Newton, factorised there) and the
    // last (quasi-Newton on that factor); passes between are the iterative solver's.
    void solvePosition(u32 pass, u32 passes);

    u32 groupCount() const
    {
        return mSolvedGroups;
    }
    u32 rowCount() const
    {
        return mSolvedRows;
    }

private:
    static constexpr u32 MaxRowParts = ArticulationRow::MaxParts;
    // A row over up to MaxRowParts bodies: how it is measured (jLinear,
    // jAngular) and M^-1 times its impulse (wLinear, wAngular) per body, by
    // slot in the group (-1: not moved).
    struct Row
    {
        u32 parts = 0;
        s32 slot[MaxRowParts];
        Math::vec3 jLinear[MaxRowParts];
        Math::vec3 jAngular[MaxRowParts];
        Math::vec3 wLinear[MaxRowParts];
        Math::vec3 wAngular[MaxRowParts];
        f32 error = 0.0f;
    };
    struct Motor
    {
        // A joint's row (joint, id), or a contact's (contact, point, axis:
        // 0 normal, 1 and 2 the tangents).
        Joint* joint = nullptr;
        u32 id = 0;
        Contact* contact = nullptr;
        u32 point = 0;
        u32 axis = 0;
        bool wantsGain = false;
        Row row;
        u32 compensation = 0;
        u32 response = 0;
    };
    struct Group
    {
        u32 firstJoint = 0;
        u32 jointCount = 0;
        u32 firstBody = 0;
        u32 bodyCount = 0;
        u32 firstRow = 0;
        u32 rowCount = 0;
        u32 firstMotor = 0;
        u32 motorCount = 0;
        u32 jointMotorCount = 0;
        u32 matrix = 0;
        u32 diagonal = 0;
        u32 coupling = 0;
        u32 positionMatrix = 0;
        u32 positionDiagonal = 0;
        u32 positionRows = 0;
        bool positionFactored = false;
        bool solved = false;
        bool velocitySolved = false;
    };

    s32 slotOf(const RigidBody* body) const;
    u32 findRoot(u32 index);
    void addPart(Row& row, RigidBody* body, const Math::vec3& linear, const Math::vec3& angular,
                 const Math::vec3& forceLinear, const Math::vec3& forceAngular) const;
    void makeRow(Row& row, RigidBody* a, RigidBody* b, const JointRow& jacobian) const;
    void makeRow(Row& row, const ArticulationRow& wide) const;
    void addImpulse(const Row& row, f32 impulse, Math::vec3* linear, Math::vec3* angular) const;
    f32 rowVelocity(const Row& row) const;
    f64 rowProduct(const Row& p, const Row& q) const;
    // Dense LDL^T of the n x n matrix at `matrix` (row-major, lower part),
    // in place; rows that depend on earlier ones are dropped.
    void factor(f64* matrix, u32 n, f64* diagonal, u8* dropped, u32* first);
    void solveFactored(const f64* matrix, u32 n, const f64* diagonal, const u8* dropped, const u32* first,
                       f64* x) const;
    void buildMatrix(const Row* rows, u32 n, f64* matrix);
    void loadVelocities(const Group& group);
    void storeVelocities(const Group& group);
    bool unchanged(const Group& group) const;
    void orderJoints(const Group& group);
    void prepareGroup(Group& group, u32 groupIndex, f32 duration);
    f32 solveContactRow(const Motor& motor, f32 velocity, f32 inverseMass) const;
    void positionGroup(Group& group);
    static bool sameRow(const Row& p, const Row& q);

    ct::Vector<Joint*> mJoints;
    ct::Vector<Contact*> mAbsorbed;
    ct::Vector<u32> mAbsorbedGroup;
    f32 mBaumgarte = 0.2f;
    f32 mSlop = 0.005f;
    f32 mMaxPenetration = 0.2f;
    ct::Vector<ArticulationRow> mWide;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<u32> mParent;
    ct::Vector<u32> mRootGroup;
    HashMap<const RigidBody*, u32> mBodyIndex;
    ct::Vector<s32> mSlot;
    ct::Vector<Group> mGroups;
    ct::Vector<Joint*> mGroupJoints;
    ct::Vector<RigidBody*> mGroupBodies;
    ct::Vector<u32> mJointRowBegin;
    ct::Vector<u32> mJointRowCount;
    ct::Vector<Row> mRows;
    ct::Vector<Motor> mMotors;
    ct::Vector<f64> mMatrix;
    ct::Vector<f64> mDiagonal;
    ct::Vector<u8> mDropped;
    ct::Vector<u32> mFirst;
    ct::Vector<f64> mPositionFactors;
    ct::Vector<f64> mPositionFactorDiagonal;
    ct::Vector<Row> mPositionFactorRows;
    ct::Vector<Row> mMergedRows;
    ct::Vector<u32> mBuildBegin;
    ct::Vector<u32> mBuildEntries;
    ct::Vector<u32> mBuildFill;
    ct::Vector<u32> mOrderDegree;
    ct::Vector<u32> mOrderDistance;
    ct::Vector<u32> mOrderKey;
    ct::Vector<Math::vec3> mStoredVelocity;
    ct::Vector<u32> mSlotRowBegin;
    ct::Vector<u32> mSlotRows;
    ct::Vector<u32> mSlotFill;
    ct::Vector<u8> mPositionFactorDropped;
    ct::Vector<u32> mPositionFactorFirst;
    ct::Vector<u32> mPositionFirst;
    ct::Vector<f64> mCompensation;
    ct::Vector<Math::vec3> mResponse;
    ct::Vector<f64> mLockImpulse;
    // A_ij: the velocity of motor row i per unit impulse of motor j, the
    // locks holding - the motors' Schur complement.
    ct::Vector<f32> mCoupling;
    ct::Vector<f32> mMotorVelocity;
    ct::Vector<f32> mMotorImpulse;
    ct::Vector<Math::vec3> mVelocity;
    ct::Vector<Math::vec3> mAngularVelocity;
    ct::Vector<f64> mWork;
    ct::Vector<f64> mWork2;
    ct::Vector<Row> mPositionRows;
    ct::Vector<f64> mPositionMatrix;
    ct::Vector<f64> mPositionDiagonal;
    ct::Vector<u8> mPositionDropped;
    ct::Vector<Math::vec3> mPositionDelta;
    u32 mSolvedGroups = 0;
    u32 mSolvedRows = 0;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_ARTICULATIONSOLVER_H
