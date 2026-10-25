#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/ArticulationSolver.h"

#include "JointAxis.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr u32 kInvalid = 0xFFFFFFFFu;

// A pivot this small against its own diagonal marks a row as a combination of
// earlier ones (a closed loop of hinges has a few).
constexpr f64 kDependentPivot = 1.0e-6;

// Max sweeps of the motors' projected Gauss-Seidel per world iteration, and
// the relative change that stops it early.
constexpr u32 kMotorSweeps = 12;
constexpr f32 kMotorTolerance = 1.0e-4f;

// Max a position iteration moves any row (m or rad): a group knocked far out of
// shape closes over a few steps instead of snapping.
constexpr f32 kMaxCorrection = 0.25f;
// A position row counts as the factorised one while its Jacobian is within
// this share of it.
constexpr f32 kSameRow = 0.02f;

bool movable(const RigidBody* body)
{
    return body && body->isDynamic();
}

bool awakeDynamic(const RigidBody* body)
{
    return body && body->isDynamic() && body->awake();
}

f64 dot3(const Math::vec3& a, const Math::vec3& b)
{
    return static_cast<f64>(a.x) * b.x + static_cast<f64>(a.y) * b.y + static_cast<f64>(a.z) * b.z;
}

// By-component vector arithmetic for the inner loops, free of calls.
inline f32 dotf(const Math::vec3& a, const Math::vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline void addScaled(Math::vec3& to, const Math::vec3& v, f32 scale)
{
    to.x += v.x * scale;
    to.y += v.y * scale;
    to.z += v.z * scale;
}

} // namespace

s32 ArticulationSolver::slotOf(const RigidBody* body) const
{
    if (!movable(body))
        return -1;
    const u32* index = mBodyIndex.find(body);
    return index ? mSlot[*index] : -1;
}

u32 ArticulationSolver::findRoot(u32 index)
{
    while (mParent[index] != index)
    {
        mParent[index] = mParent[mParent[index]];
        index = mParent[index];
    }
    return index;
}

void ArticulationSolver::addPart(Row& row, RigidBody* body, const Math::vec3& linear, const Math::vec3& angular,
                                 const Math::vec3& forceLinear, const Math::vec3& forceAngular) const
{
    const s32 slot = slotOf(body);
    if (slot < 0 || row.parts >= MaxRowParts)
        return;
    for (u32 i = 0; i < row.parts; ++i)
        if (row.slot[i] == slot)
        {
            row.jLinear[i] += linear;
            row.jAngular[i] += angular;
            row.wLinear[i] += forceLinear * body->inverseMass();
            row.wAngular[i] += body->inverseInertiaTensorWorld() * forceAngular;
            return;
        }
    const u32 i = row.parts++;
    row.slot[i] = slot;
    row.jLinear[i] = linear;
    row.jAngular[i] = angular;
    row.wLinear[i] = forceLinear * body->inverseMass();
    row.wAngular[i] = body->inverseInertiaTensorWorld() * forceAngular;
}

void ArticulationSolver::makeRow(Row& row, RigidBody* a, RigidBody* b, const JointRow& jacobian) const
{
    row.parts = 0;
    row.error = 0.0f;
    addPart(row, a, jacobian.linearA, jacobian.angularA, jacobian.linearA, jacobian.angularA);
    addPart(row, b, jacobian.linearB, jacobian.angularB, jacobian.linearB, jacobian.angularB);
}

void ArticulationSolver::makeRow(Row& row, const ArticulationRow& wide) const
{
    row.parts = 0;
    row.error = 0.0f;
    for (u32 i = 0; i < wide.parts && i < ArticulationRow::MaxParts; ++i)
        addPart(row, wide.body[i], wide.linear[i], wide.angular[i], wide.asymmetric ? wide.forceLinear[i] : wide.linear[i],
                wide.asymmetric ? wide.forceAngular[i] : wide.angular[i]);
}

f32 ArticulationSolver::rowVelocity(const Row& row) const
{
    f32 velocity = 0.0f;
    const Math::vec3* linear = mVelocity.data();
    const Math::vec3* angular = mAngularVelocity.data();
    for (u32 i = 0; i < row.parts; ++i)
        velocity += dotf(row.jLinear[i], linear[row.slot[i]]) + dotf(row.jAngular[i], angular[row.slot[i]]);
    return velocity;
}

void ArticulationSolver::addImpulse(const Row& row, f32 impulse, Math::vec3* linear, Math::vec3* angular) const
{
    for (u32 i = 0; i < row.parts; ++i)
    {
        addScaled(linear[row.slot[i]], row.wLinear[i], impulse);
        addScaled(angular[row.slot[i]], row.wAngular[i], impulse);
    }
}

// J_p M^-1 F_q^T: rows meet only on the bodies they share.
f64 ArticulationSolver::rowProduct(const Row& p, const Row& q) const
{
    f64 sum = 0.0;
    for (u32 i = 0; i < p.parts; ++i)
        for (u32 j = 0; j < q.parts; ++j)
            if (p.slot[i] == q.slot[j])
                sum += dot3(p.jLinear[i], q.wLinear[j]) + dot3(p.jAngular[i], q.wAngular[j]);
    return sum;
}

// K = J M^-1 F^T, lower triangle, body by body: each body adds the products of
// the rows on it, so a hub with many rows costs only their pairs.
void ArticulationSolver::buildMatrix(const Row* rows, u32 n, f64* matrix)
{
    for (u32 i = 0; i < n; ++i)
        for (u32 j = 0; j <= i; ++j)
            matrix[i * n + j] = 0.0;
    u32 slots = 0;
    for (u32 i = 0; i < n; ++i)
        for (u32 k = 0; k < rows[i].parts; ++k)
            slots = Math::max(slots, static_cast<u32>(rows[i].slot[k]) + 1);
    mBuildBegin.resize(slots + 1);
    for (u32 b = 0; b <= slots; ++b)
        mBuildBegin[b] = 0;
    for (u32 i = 0; i < n; ++i)
        for (u32 k = 0; k < rows[i].parts; ++k)
            ++mBuildBegin[rows[i].slot[k] + 1];
    for (u32 b = 0; b < slots; ++b)
        mBuildBegin[b + 1] += mBuildBegin[b];
    mBuildEntries.resize(mBuildBegin[slots]);
    mBuildFill.resize(slots);
    for (u32 b = 0; b < slots; ++b)
        mBuildFill[b] = mBuildBegin[b];
    for (u32 i = 0; i < n; ++i)
        for (u32 k = 0; k < rows[i].parts; ++k)
            mBuildEntries[mBuildFill[rows[i].slot[k]]++] = (i << 2) | k;
    const u32* entries = mBuildEntries.data();
    for (u32 b = 0; b < slots; ++b)
        for (u32 x = mBuildBegin[b]; x < mBuildBegin[b + 1]; ++x)
        {
            const u32 i = entries[x] >> 2;
            const Row& p = rows[i];
            const u32 kp = entries[x] & 3u;
            f64* row = matrix + i * n;
            for (u32 y = mBuildBegin[b]; y <= x; ++y)
            {
                const u32 j = entries[y] >> 2;
                const Row& q = rows[j];
                const u32 kq = entries[y] & 3u;
                row[j] += dot3(p.jLinear[kp], q.wLinear[kq]) + dot3(p.jAngular[kp], q.wAngular[kq]);
            }
        }
}

// LDL^T in the lower triangle within each row's envelope: row i's first
// non-zero column first[i] bounds its fill-in, so a chain factorises in about
// its bandwidth squared per row. A row whose pivot comes out at (almost)
// nothing is a combination of earlier rows and is dropped: zero column, no
// impulse from solveFactored().
void ArticulationSolver::factor(f64* m, u32 n, f64* d, u8* dropped, u32* first)
{
    for (u32 i = 0; i < n; ++i)
    {
        u32 f = i;
        for (u32 j = 0; j < i; ++j)
            if (m[i * n + j] != 0.0)
            {
                f = j;
                break;
            }
        first[i] = f;
    }
    for (u32 j = 0; j < n; ++j)
    {
        const f64 diagonal = m[j * n + j];
        f64 pivot = diagonal;
        const f64* rowJ = m + j * n;
        for (u32 k = first[j]; k < j; ++k)
            pivot -= rowJ[k] * rowJ[k] * d[k];
        if (!(diagonal > 0.0) || !(pivot > kDependentPivot * diagonal) || !isfinite(pivot))
        {
            dropped[j] = 1;
            d[j] = 0.0;
            for (u32 i = j + 1; i < n; ++i)
                if (first[i] <= j)
                    m[i * n + j] = 0.0;
            continue;
        }
        dropped[j] = 0;
        d[j] = pivot;
        const f64 inverse = 1.0 / pivot;
        for (u32 i = j + 1; i < n; ++i)
        {
            if (first[i] > j)
                continue;
            f64* rowI = m + i * n;
            f64 sum = rowI[j];
            const u32 start = first[i] > first[j] ? first[i] : first[j];
            for (u32 k = start; k < j; ++k)
                sum -= rowI[k] * rowJ[k] * d[k];
            rowI[j] = sum * inverse;
        }
    }
}

void ArticulationSolver::solveFactored(const f64* m, u32 n, const f64* d, const u8* dropped, const u32* first,
                                       f64* x) const
{
    // L y = b from the right-hand side's first non-zero; everything before it
    // stays zero.
    u32 start = 0;
    while (start < n && x[start] == 0.0)
        ++start;
    for (u32 i = start; i < n; ++i)
    {
        if (dropped[i])
        {
            x[i] = 0.0;
            continue;
        }
        f64 sum = x[i];
        const f64* row = m + i * n;
        for (u32 k = first[i] > start ? first[i] : start; k < i; ++k)
            sum -= row[k] * x[k];
        x[i] = sum;
    }
    for (u32 i = 0; i < n; ++i)
        x[i] = dropped[i] ? 0.0 : x[i] / d[i];
    // L^T x = z from the bottom, each finished x_i taken out of its envelope's rows.
    for (u32 i = n; i-- > 0;)
    {
        if (dropped[i])
            continue;
        const f64* row = m + i * n;
        const f64 value = x[i];
        for (u32 k = first[i]; k < i; ++k)
            if (!dropped[k])
                x[k] -= row[k] * value;
    }
}

void ArticulationSolver::prepare(Joint* const* joints, u32 count, Contact* contacts, u32 contactCount, f32 duration,
                                 f32 baumgarte, f32 slop, f32 maxPenetration)
{
    mBaumgarte = baumgarte;
    mSlop = slop;
    mMaxPenetration = maxPenetration;
    mAbsorbed.clear();
    mAbsorbedGroup.clear();
    for (u32 c = 0; c < contactCount; ++c)
        contacts[c].articulated = false;
    mSolvedGroups = 0;
    mSolvedRows = 0;
    mJoints.clear();
    mBodies.clear();
    mParent.clear();
    mBodyIndex.clear();
    mSlot.clear();
    mGroups.clear();
    mGroupJoints.clear();
    mGroupBodies.clear();
    mRows.clear();
    mMotors.clear();
    mJointRowBegin.clear();
    mJointRowCount.clear();
    mMatrix.clear();
    mDiagonal.clear();
    mDropped.clear();
    mCompensation.clear();
    mResponse.clear();
    mLockImpulse.clear();
    mCoupling.clear();
    mFirst.clear();
    mPositionFactors.clear();
    mPositionFactorDiagonal.clear();
    mPositionFactorDropped.clear();
    mPositionFactorFirst.clear();
    mPositionFactorRows.clear();

    RigidBody* bodies[Joint::MaxArticulationBodies];
    for (u32 i = 0; i < count; ++i)
    {
        Joint* joint = joints[i];
        if (!joint)
            continue;
        joint->mInArticulation = false;
        if (!joint->enabled() || !joint->articulated() || !joint->supportsArticulation())
            continue;
        const u32 n = Math::min(joint->articulationBodies(bodies), Joint::MaxArticulationBodies);
        // Solved only while something in it can move, as the iterative solver does.
        bool awake = false;
        for (u32 k = 0; k < n; ++k)
            awake = awake || awakeDynamic(bodies[k]);
        if (!awake)
            continue;
        mJoints.push_back(joint);
        for (u32 k = 0; k < n; ++k)
        {
            RigidBody* body = bodies[k];
            if (!movable(body) || mBodyIndex.find(body))
                continue;
            mBodyIndex[body] = static_cast<u32>(mBodies.size());
            mParent.push_back(static_cast<u32>(mBodies.size()));
            mBodies.push_back(body);
        }
    }
    if (mJoints.empty())
        return;

    // Groups: joints joined through the dynamic bodies they share (a static or
    // kinematic body joins nothing).
    for (Joint* joint : mJoints)
    {
        const u32 n = Math::min(joint->articulationBodies(bodies), Joint::MaxArticulationBodies);
        s32 first = -1;
        for (u32 k = 0; k < n; ++k)
        {
            const u32* index = movable(bodies[k]) ? mBodyIndex.find(bodies[k]) : nullptr;
            if (!index)
                continue;
            if (first < 0)
            {
                first = static_cast<s32>(*index);
                continue;
            }
            const u32 rootA = findRoot(static_cast<u32>(first));
            const u32 rootB = findRoot(*index);
            if (rootA != rootB)
                mParent[rootB] = rootA;
        }
    }

    // Bodies laid out group by group; mSlot is each body's place in its group.
    mSlot.resize(mBodies.size());
    ct::Vector<u32>& rootGroup = mRootGroup;
    rootGroup.resize(mBodies.size());
    for (u32 i = 0; i < mBodies.size(); ++i)
        rootGroup[i] = kInvalid;
    for (u32 i = 0; i < mBodies.size(); ++i)
    {
        const u32 root = findRoot(i);
        if (rootGroup[root] == kInvalid)
        {
            rootGroup[root] = static_cast<u32>(mGroups.size());
            mGroups.push_back(Group());
        }
    }
    for (u32 g = 0; g < mGroups.size(); ++g)
    {
        Group& group = mGroups[g];
        group.firstBody = static_cast<u32>(mGroupBodies.size());
        for (u32 i = 0; i < mBodies.size(); ++i)
            if (rootGroup[findRoot(i)] == g)
            {
                mSlot[i] = static_cast<s32>(mGroupBodies.size() - group.firstBody);
                mGroupBodies.push_back(mBodies[i]);
            }
        group.bodyCount = static_cast<u32>(mGroupBodies.size()) - group.firstBody;
    }
    for (u32 g = 0; g < mGroups.size(); ++g)
    {
        Group& group = mGroups[g];
        group.firstJoint = static_cast<u32>(mGroupJoints.size());
        for (Joint* joint : mJoints)
        {
            const u32 n = Math::min(joint->articulationBodies(bodies), Joint::MaxArticulationBodies);
            const u32* index = nullptr;
            for (u32 k = 0; k < n && !index; ++k)
                index = movable(bodies[k]) ? mBodyIndex.find(bodies[k]) : nullptr;
            if (index && rootGroup[findRoot(*index)] == g)
                mGroupJoints.push_back(joint);
        }
        group.jointCount = static_cast<u32>(mGroupJoints.size()) - group.firstJoint;
    }

    // Contacts the groups make with what does not move (or among themselves)
    // are theirs; a contact with a dynamic body outside the groups stays with
    // the contact solver.
    for (u32 c = 0; c < contactCount; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        const u32* a = movable(contact.a) ? mBodyIndex.find(contact.a) : nullptr;
        const u32* b = movable(contact.b) ? mBodyIndex.find(contact.b) : nullptr;
        const u32 groupA = a ? rootGroup[findRoot(*a)] : kInvalid;
        const u32 groupB = b ? rootGroup[findRoot(*b)] : kInvalid;
        u32 group = kInvalid;
        if (groupA != kInvalid && !movable(contact.b))
            group = groupA;
        else if (groupB != kInvalid && !movable(contact.a))
            group = groupB;
        else if (groupA != kInvalid && groupA == groupB)
            group = groupA;
        if (group == kInvalid)
            continue;
        mAbsorbed.push_back(&contact);
        mAbsorbedGroup.push_back(group);
    }

    mJointRowBegin.resize(mGroupJoints.size());
    mJointRowCount.resize(mGroupJoints.size());
    for (u32 g = 0; g < mGroups.size(); ++g)
        prepareGroup(mGroups[g], g, duration);
}

// One contact row in the motors' Gauss-Seidel: the normal pushes (>= 0)
// towards the contact's target separation speed, each tangent holds the
// sliding within the friction of the normal it has now.
f32 ArticulationSolver::solveContactRow(const Motor& motor, f32 velocity, f32 inverseMass) const
{
    if (!(inverseMass > 0.0f))
        return 0.0f;
    ContactPoint& point = motor.contact->manifold.points[motor.point];
    if (motor.axis == 0)
    {
        const f32 previous = point.normalImpulse;
        const f32 total = Math::max(previous - (velocity - point.velocityBias) / inverseMass, 0.0f);
        point.normalImpulse = total;
        return total - previous;
    }
    const f32 limit = motor.contact->friction * point.normalImpulse;
    f32& accumulated = point.tangentImpulse[motor.axis - 1];
    const f32 previous = accumulated;
    const f32 total = Math::clamp(previous - velocity / inverseMass, -limit, limit);
    accumulated = total;
    return total - previous;
}

// The joints of a group in elimination order for the envelope factor: furthest
// from the hub (the body most joints meet) first, the hub's own last. A tree
// eliminated from its leaves factorises without fill, and the dense block at
// each hub sits at the end. Distances count joints along shared bodies.
void ArticulationSolver::orderJoints(const Group& group)
{
    const u32 count = group.jointCount;
    if (count < 3)
        return;
    Joint** joints = mGroupJoints.data() + group.firstJoint;
    RigidBody* bodies[Joint::MaxArticulationBodies];
    const u32 bodyCount = group.bodyCount;
    mOrderDegree.resize(bodyCount);
    mOrderDistance.resize(bodyCount);
    for (u32 b = 0; b < bodyCount; ++b)
    {
        mOrderDegree[b] = 0;
        mOrderDistance[b] = kInvalid;
    }
    for (u32 j = 0; j < count; ++j)
    {
        const u32 n = Math::min(joints[j]->articulationBodies(bodies), Joint::MaxArticulationBodies);
        for (u32 k = 0; k < n; ++k)
        {
            const s32 slot = slotOf(bodies[k]);
            if (slot >= 0)
                ++mOrderDegree[static_cast<u32>(slot)];
        }
    }
    u32 hub = 0;
    for (u32 b = 1; b < bodyCount; ++b)
        if (mOrderDegree[b] > mOrderDegree[hub])
            hub = b;
    // Distances from the hub, relaxed until settled.
    mOrderDistance[hub] = 0;
    for (bool changed = true; changed;)
    {
        changed = false;
        for (u32 j = 0; j < count; ++j)
        {
            const u32 n = Math::min(joints[j]->articulationBodies(bodies), Joint::MaxArticulationBodies);
            u32 nearest = kInvalid;
            for (u32 k = 0; k < n; ++k)
            {
                const s32 slot = slotOf(bodies[k]);
                if (slot >= 0)
                    nearest = Math::min(nearest, mOrderDistance[static_cast<u32>(slot)]);
            }
            if (nearest == kInvalid)
                continue;
            for (u32 k = 0; k < n; ++k)
            {
                const s32 slot = slotOf(bodies[k]);
                if (slot >= 0 && mOrderDistance[static_cast<u32>(slot)] > nearest + 1)
                {
                    mOrderDistance[static_cast<u32>(slot)] = nearest + 1;
                    changed = true;
                }
            }
        }
    }
    mOrderKey.resize(count);
    for (u32 j = 0; j < count; ++j)
    {
        const u32 n = Math::min(joints[j]->articulationBodies(bodies), Joint::MaxArticulationBodies);
        u32 furthest = 0;
        for (u32 k = 0; k < n; ++k)
        {
            const s32 slot = slotOf(bodies[k]);
            if (slot >= 0 && mOrderDistance[static_cast<u32>(slot)] != kInvalid)
                furthest = Math::max(furthest, mOrderDistance[static_cast<u32>(slot)]);
        }
        mOrderKey[j] = furthest;
    }
    for (u32 j = 1; j < count; ++j)
    {
        Joint* joint = joints[j];
        const u32 key = mOrderKey[j];
        u32 i = j;
        while (i > 0 && mOrderKey[i - 1] < key)
        {
            joints[i] = joints[i - 1];
            mOrderKey[i] = mOrderKey[i - 1];
            --i;
        }
        joints[i] = joint;
        mOrderKey[i] = key;
    }
}

void ArticulationSolver::prepareGroup(Group& group, u32 groupIndex, f32 duration)
{
    (void)duration;
    group.solved = false;
    group.firstRow = static_cast<u32>(mRows.size());
    group.firstMotor = static_cast<u32>(mMotors.size());

    orderJoints(group);

    // Slots are relative to the group.
    JointRow jacobians[Joint::MaxArticulationRows];
    f32 errors[Joint::MaxArticulationRows];
    for (u32 j = 0; j < group.jointCount; ++j)
    {
        Joint* joint = mGroupJoints[group.firstJoint + j];
        const u32 n = Math::min(joint->articulationLocks(jacobians, errors), Joint::MaxArticulationRows);
        mJointRowBegin[group.firstJoint + j] = static_cast<u32>(mRows.size()) - group.firstRow;
        mJointRowCount[group.firstJoint + j] = n;
        for (u32 r = 0; r < n; ++r)
        {
            Row row;
            makeRow(row, joint->bodyA(), joint->bodyB(), jacobians[r]);
            row.error = errors[r];
            mRows.push_back(row);
        }
    }
    group.rowCount = static_cast<u32>(mRows.size()) - group.firstRow;
    if (group.rowCount > MaxGroupRows)
    {
        // Too big for a dense solve: the iterative solver keeps it.
        mRows.resize(group.firstRow);
        group.rowCount = 0;
        return;
    }

    ArticulationMotor motors[Joint::MaxArticulationRows];
    mWide.resize(Joint::MaxArticulationRows);
    for (u32 j = 0; j < group.jointCount; ++j)
    {
        Joint* joint = mGroupJoints[group.firstJoint + j];
        for (ArticulationMotor& motor : motors)
            motor.wantsGain = false;
        const u32 n = Math::min(joint->articulationMotors(motors), Joint::MaxArticulationRows);
        for (u32 m = 0; m < n; ++m)
        {
            Motor motor;
            motor.joint = joint;
            motor.id = motors[m].id;
            motor.wantsGain = motors[m].wantsGain;
            makeRow(motor.row, joint->bodyA(), joint->bodyB(), motors[m].row);
            mMotors.push_back(motor);
        }
        const u32 wide = Math::min(joint->articulationWideMotors(mWide.data()), Joint::MaxArticulationRows);
        for (u32 m = 0; m < wide; ++m)
        {
            Motor motor;
            motor.joint = joint;
            motor.id = mWide[m].id;
            makeRow(motor.row, mWide[m]);
            mMotors.push_back(motor);
        }
    }
    group.jointMotorCount = static_cast<u32>(mMotors.size()) - group.firstMotor;
    // The ground contacts: three rows a point, normal first.
    for (u32 c = 0; c < mAbsorbed.size(); ++c)
    {
        if (mAbsorbedGroup[c] != groupIndex)
            continue;
        Contact& contact = *mAbsorbed[c];
        contact.articulated = true;
        const ContactManifold& manifold = contact.manifold;
        const Math::vec3 directions[3] = {manifold.normal, manifold.tangent[0], manifold.tangent[1]};
        for (u32 k = 0; k < manifold.count; ++k)
        {
            const Math::vec3& p = manifold.points[k].position;
            const Math::vec3 armA = p - contact.a->position();
            const Math::vec3 armB = p - contact.b->position();
            for (u32 axis = 0; axis < 3; ++axis)
            {
                Motor motor;
                motor.contact = &contact;
                motor.point = k;
                motor.axis = axis;
                makeRow(motor.row, contact.a, contact.b, detail::linearRow(armA, armB, directions[axis]));
                mMotors.push_back(motor);
            }
        }
    }
    group.motorCount = static_cast<u32>(mMotors.size()) - group.firstMotor;

    const u32 n = group.rowCount;
    group.solved = true;
    ++mSolvedGroups;
    mSolvedRows += n;
    for (u32 j = 0; j < group.jointCount; ++j)
        mGroupJoints[group.firstJoint + j]->mInArticulation = true;

    group.matrix = static_cast<u32>(mMatrix.size());
    mMatrix.resize(mMatrix.size() + n * n);
    const u32 diagonal = static_cast<u32>(mDiagonal.size());
    group.diagonal = diagonal;
    mDiagonal.resize(diagonal + n);
    mDropped.resize(diagonal + n);
    mLockImpulse.resize(diagonal + n);
    for (u32 i = 0; i < n; ++i)
        mLockImpulse[diagonal + i] = 0.0;
    const Row* rows = mRows.data() + group.firstRow;
    f64* matrix = mMatrix.data() + group.matrix;
    buildMatrix(rows, n, matrix);
    mFirst.resize(diagonal + n);
    factor(matrix, n, mDiagonal.data() + diagonal, mDropped.data() + diagonal, mFirst.data() + diagonal);

    // Lock rows on each body of the group: a motor meets only the locks on its bodies.
    mSlotRowBegin.resize(group.bodyCount + 1);
    for (u32 b = 0; b <= group.bodyCount; ++b)
        mSlotRowBegin[b] = 0;
    for (u32 i = 0; i < n; ++i)
        for (u32 k = 0; k < rows[i].parts; ++k)
            ++mSlotRowBegin[rows[i].slot[k] + 1];
    for (u32 b = 0; b < group.bodyCount; ++b)
        mSlotRowBegin[b + 1] += mSlotRowBegin[b];
    mSlotRows.resize(mSlotRowBegin[group.bodyCount]);
    mSlotFill.resize(group.bodyCount);
    for (u32 b = 0; b < group.bodyCount; ++b)
        mSlotFill[b] = mSlotRowBegin[b];
    for (u32 i = 0; i < n; ++i)
        for (u32 k = 0; k < rows[i].parts; ++k)
            mSlotRows[mSlotFill[rows[i].slot[k]]++] = i;

    // The motors: each row's lock compensation x = K^-1 c, c_i = J_i M^-1
    // F_m^T (what one unit of the motor's impulse does to lock i), and the
    // velocity change one unit makes on each body of the group with the
    // compensation in: M^-1 (F_m^T - J_lock^T x).
    mWork.resize(n + group.motorCount);
    for (u32 m = 0; m < group.motorCount; ++m)
    {
        Motor& motor = mMotors[group.firstMotor + m];
        motor.compensation = static_cast<u32>(mCompensation.size());
        mCompensation.resize(mCompensation.size() + n);
        f64* x = mCompensation.data() + motor.compensation;
        for (u32 i = 0; i < n; ++i)
            x[i] = 0.0;
        for (u32 k = 0; k < motor.row.parts; ++k)
        {
            const s32 slot = motor.row.slot[k];
            for (u32 r = mSlotRowBegin[slot]; r < mSlotRowBegin[slot + 1]; ++r)
            {
                const u32 i = mSlotRows[r];
                // A row on two of the motor's bodies is met twice; once is its whole product.
                if (x[i] == 0.0)
                    x[i] = rowProduct(rows[i], motor.row);
            }
        }
        solveFactored(matrix, n, mDiagonal.data() + diagonal, mDropped.data() + diagonal, mFirst.data() + diagonal, x);

        motor.response = static_cast<u32>(mResponse.size());
        mResponse.resize(mResponse.size() + 2 * group.bodyCount);
        Math::vec3* response = mResponse.data() + motor.response;
        for (u32 b = 0; b < 2 * group.bodyCount; ++b)
            response[b].x = response[b].y = response[b].z = 0.0f;
        for (u32 k = 0; k < motor.row.parts; ++k)
        {
            addScaled(response[2 * motor.row.slot[k]], motor.row.wLinear[k], 1.0f);
            addScaled(response[2 * motor.row.slot[k] + 1], motor.row.wAngular[k], 1.0f);
        }
        for (u32 i = 0; i < n; ++i)
        {
            const f32 weight = -static_cast<f32>(x[i]);
            if (weight == 0.0f)
                continue;
            const Row& row = rows[i];
            for (u32 k = 0; k < row.parts; ++k)
            {
                addScaled(response[2 * row.slot[k]], row.wLinear[k], weight);
                addScaled(response[2 * row.slot[k] + 1], row.wAngular[k], weight);
            }
        }
        // The motor's mass with the locks holding, 1 / (J_m r_m): J_m M^-1 F_m^T
        // less what the locks take.
        const Math::vec3* own = response;
        f64 reduced = 0.0;
        for (u32 k = 0; k < motor.row.parts; ++k)
            reduced += dot3(motor.row.jLinear[k], own[2 * motor.row.slot[k]]) +
                       dot3(motor.row.jAngular[k], own[2 * motor.row.slot[k] + 1]);
        const f64 bare = rowProduct(motor.row, motor.row);
        if (!(reduced > 1.0e-6 * bare) || !isfinite(reduced))
            reduced = bare;
        mWork[m] = reduced;
    }

    if (group.motorCount == 0)
        return;

    // The motors' coupling with the locks holding: A_ij = J_i . r_j, r_j
    // the compensated response of motor j.
    const u32 nm = group.motorCount;
    group.coupling = static_cast<u32>(mCoupling.size());
    mCoupling.resize(mCoupling.size() + nm * nm);
    f32* coupling = mCoupling.data() + group.coupling;
    for (u32 i = 0; i < nm; ++i)
    {
        const Row& row = mMotors[group.firstMotor + i].row;
        for (u32 j = 0; j < nm; ++j)
        {
            const Math::vec3* response = mResponse.data() + mMotors[group.firstMotor + j].response;
            f32 sum = 0.0f;
            for (u32 k = 0; k < row.parts; ++k)
                sum += dotf(row.jLinear[k], response[2 * row.slot[k]]) +
                       dotf(row.jAngular[k], response[2 * row.slot[k] + 1]);
            coupling[i * nm + j] = sum;
        }
    }

    // Inertia each drive's gains are scaled by: its row's Schur complement
    // against every other row of the group, 1 / (K_all^-1)_mm. With the other
    // drives holding, an axis moves everything outboard of it as one body
    // (diagonal of the joint-space mass matrix).
    const u32 jointMotors = group.jointMotorCount;
    bool gains = false;
    for (u32 m = 0; m < jointMotors; ++m)
        gains = gains || mMotors[group.firstMotor + m].wantsGain;
    if (!gains)
    {
        for (u32 m = 0; m < jointMotors; ++m)
        {
            Motor& motor = mMotors[group.firstMotor + m];
            motor.joint->setArticulationMotorMass(motor.id, static_cast<f32>(mWork[m]), static_cast<f32>(mWork[m]));
        }
        return;
    }
    const u32 total = n + jointMotors;
    mPositionRows.clear();
    for (u32 i = 0; i < n; ++i)
        mPositionRows.push_back(rows[i]);
    for (u32 m = 0; m < jointMotors; ++m)
        mPositionRows.push_back(mMotors[group.firstMotor + m].row);
    mPositionMatrix.resize(total * total);
    mPositionDiagonal.resize(total);
    mPositionDropped.resize(total);
    buildMatrix(mPositionRows.data(), total, mPositionMatrix.data());
    mPositionFirst.resize(total);
    factor(mPositionMatrix.data(), total, mPositionDiagonal.data(), mPositionDropped.data(), mPositionFirst.data());
    mWork2.resize(total);
    for (u32 m = 0; m < jointMotors; ++m)
    {
        Motor& motor = mMotors[group.firstMotor + m];
        const f64 reduced = mWork[m];
        f64 gain = reduced;
        if (!mPositionDropped[n + m])
        {
            for (u32 i = 0; i < total; ++i)
                mWork2[i] = i == n + m ? 1.0 : 0.0;
            solveFactored(mPositionMatrix.data(), total, mPositionDiagonal.data(), mPositionDropped.data(),
                          mPositionFirst.data(), mWork2.data());
            const f64 inverseDiagonal = mWork2[n + m];
            if (inverseDiagonal > 0.0 && isfinite(inverseDiagonal))
                gain = Math::min(1.0 / inverseDiagonal, reduced);
        }
        motor.joint->setArticulationMotorMass(motor.id, static_cast<f32>(reduced), static_cast<f32>(gain));
    }
}

void ArticulationSolver::loadVelocities(const Group& group)
{
    mVelocity.resize(group.bodyCount);
    mAngularVelocity.resize(group.bodyCount);
    for (u32 b = 0; b < group.bodyCount; ++b)
    {
        const RigidBody* body = mGroupBodies[group.firstBody + b];
        mVelocity[b] = body->velocity();
        mAngularVelocity[b] = body->angularVelocity();
    }
}

void ArticulationSolver::storeVelocities(const Group& group)
{
    mStoredVelocity.resize(2 * mGroupBodies.size());
    Math::vec3* stored = mStoredVelocity.data() + 2 * group.firstBody;
    for (u32 b = 0; b < group.bodyCount; ++b)
    {
        RigidBody* body = mGroupBodies[group.firstBody + b];
        body->setVelocity(mVelocity[b]);
        body->setAngularVelocity(mAngularVelocity[b]);
        stored[2 * b] = body->velocity();
        stored[2 * b + 1] = body->angularVelocity();
    }
}

bool ArticulationSolver::unchanged(const Group& group) const
{
    const Math::vec3* stored = mStoredVelocity.data() + 2 * group.firstBody;
    for (u32 b = 0; b < group.bodyCount; ++b)
    {
        const RigidBody* body = mGroupBodies[group.firstBody + b];
        const Math::vec3& v = body->velocity();
        const Math::vec3& w = body->angularVelocity();
        const Math::vec3& sv = stored[2 * b];
        const Math::vec3& sw = stored[2 * b + 1];
        if (v.x != sv.x || v.y != sv.y || v.z != sv.z || w.x != sw.x || w.y != sw.y || w.z != sw.z)
            return false;
    }
    return true;
}

void ArticulationSolver::solveVelocity()
{
    for (Group& group : mGroups)
    {
        if (!group.solved)
            continue;
        const u32 diagonal = group.diagonal;
        const u32 n = group.rowCount;
        const Row* rows = mRows.data() + group.firstRow;
        const f64* matrix = mMatrix.data() + group.matrix;
        // Nothing touched the group's bodies since its last solve this step, so
        // the locks hold and the motors have settled.
        if (group.velocitySolved && unchanged(group))
            continue;
        loadVelocities(group);

        // The locks, exactly: K lambda = -J v.
        mWork.resize(n);
        for (u32 i = 0; i < n; ++i)
            mWork[i] = -static_cast<f64>(rowVelocity(rows[i]));
        solveFactored(matrix, n, mDiagonal.data() + diagonal, mDropped.data() + diagonal, mFirst.data() + diagonal,
                      mWork.data());
        for (u32 i = 0; i < n; ++i)
        {
            const f32 lambda = static_cast<f32>(mWork[i]);
            if (lambda == 0.0f)
                continue;
            mLockImpulse[diagonal + i] += mWork[i];
            addImpulse(rows[i], lambda, mVelocity.data(), mAngularVelocity.data());
        }

        // The bounded rows in the motor space the locks leave: projected
        // Gauss-Seidel on their coupling (each joint applies its own rule and
        // bounds), row velocities tracked through A; then the impulses go into
        // the bodies together with the lock impulses that keep the locks satisfied.
        const u32 nm = group.motorCount;
        if (nm > 0)
        {
            mMotorVelocity.resize(nm);
            mMotorImpulse.resize(nm);
            for (u32 m = 0; m < nm; ++m)
            {
                mMotorVelocity[m] = rowVelocity(mMotors[group.firstMotor + m].row);
                mMotorImpulse[m] = 0.0f;
            }
            const f32* coupling = mCoupling.data() + group.coupling;
            f32* motorVelocity = mMotorVelocity.data();
            f32* motorImpulse = mMotorImpulse.data();
            for (u32 sweep = 0; sweep < kMotorSweeps; ++sweep)
            {
                f32 largest = 0.0f;
                f32 scale = 0.0f;
                for (u32 m = 0; m < nm; ++m)
                {
                    const Motor& motor = mMotors[group.firstMotor + m];
                    const f32 impulse = motor.joint
                                            ? motor.joint->solveArticulationMotor(motor.id, motorVelocity[m])
                                            : solveContactRow(motor, motorVelocity[m], coupling[m * nm + m]);
                    if (impulse == 0.0f || !isfinite(impulse))
                        continue;
                    motorImpulse[m] += impulse;
                    const f32* column = coupling + m;
                    for (u32 k = 0; k < nm; ++k)
                        motorVelocity[k] += column[k * nm] * impulse;
                    largest = Math::max(largest, fabsf(impulse));
                    scale = Math::max(scale, fabsf(motorImpulse[m]));
                }
                if (largest <= kMotorTolerance * scale)
                    break;
            }
            for (u32 m = 0; m < nm; ++m)
            {
                const f32 impulse = mMotorImpulse[m];
                if (impulse == 0.0f)
                    continue;
                const Motor& motor = mMotors[group.firstMotor + m];
                const Math::vec3* response = mResponse.data() + motor.response;
                Math::vec3* linear = mVelocity.data();
                Math::vec3* angular = mAngularVelocity.data();
                for (u32 b = 0; b < group.bodyCount; ++b)
                {
                    addScaled(linear[b], response[2 * b], impulse);
                    addScaled(angular[b], response[2 * b + 1], impulse);
                }
                const f64* x = mCompensation.data() + motor.compensation;
                f64* lock = mLockImpulse.data() + diagonal;
                for (u32 i = 0; i < n; ++i)
                    lock[i] -= x[i] * impulse;
            }
        }
        storeVelocities(group);
        group.velocitySolved = true;
    }
}

void ArticulationSolver::finishVelocity()
{
    f32 impulses[Joint::MaxArticulationRows];
    for (const Group& group : mGroups)
    {
        if (!group.solved)
            continue;
        const u32 diagonal = group.diagonal;
        for (u32 j = 0; j < group.jointCount; ++j)
        {
            const u32 begin = mJointRowBegin[group.firstJoint + j];
            const u32 n = mJointRowCount[group.firstJoint + j];
            if (n == 0)
                continue;
            for (u32 r = 0; r < n; ++r)
                impulses[r] = static_cast<f32>(mLockImpulse[diagonal + begin + r]);
            mGroupJoints[group.firstJoint + j]->addArticulationLockImpulses(impulses);
        }
    }
}

bool ArticulationSolver::sameRow(const Row& p, const Row& q)
{
    if (p.parts != q.parts)
        return false;
    for (u32 k = 0; k < p.parts; ++k)
    {
        if (p.slot[k] != q.slot[k])
            return false;
        const f32 scale = Math::dot(q.jLinear[k], q.jLinear[k]) + Math::dot(q.jAngular[k], q.jAngular[k]);
        const Math::vec3 linear = p.jLinear[k] - q.jLinear[k];
        const Math::vec3 angular = p.jAngular[k] - q.jAngular[k];
        const f32 change = Math::dot(linear, linear) + Math::dot(angular, angular);
        if (!(change <= kSameRow * kSameRow * scale))
            return false;
    }
    return true;
}

void ArticulationSolver::positionGroup(Group& group)
{
    // Rows at the current pose: the locks, then rows a joint wants held at this
    // level (rigid servos, limits and stops it is beyond, a stretched rope).
    mPositionRows.clear();
    JointRow jacobians[Joint::MaxArticulationRows];
    f32 errors[Joint::MaxArticulationRows];
    f32 largest = 0.0f;
    mWide.resize(Joint::MaxArticulationRows);
    for (u32 j = 0; j < group.jointCount; ++j)
    {
        Joint* joint = mGroupJoints[group.firstJoint + j];
        for (u32 pass = 0; pass < 3; ++pass)
        {
            u32 n = 0;
            if (pass == 0)
                n = joint->articulationLocks(jacobians, errors);
            else if (pass == 1)
                n = joint->articulationPositionRows(jacobians, errors);
            else
                n = joint->articulationWidePositionRows(mWide.data(), errors);
            n = Math::min(n, Joint::MaxArticulationRows);
            for (u32 r = 0; r < n; ++r)
            {
                if (!isfinite(errors[r]))
                    continue;
                Row row;
                if (pass < 2)
                    makeRow(row, joint->bodyA(), joint->bodyB(), jacobians[r]);
                else
                    makeRow(row, mWide[r]);
                row.error = errors[r];
                largest = Math::max(largest, fabsf(errors[r]));
                mPositionRows.push_back(row);
            }
        }
    }
    // Ground contacts: overlap left past the slop, a share per pass (the
    // contact solver's baumgarte). A point within the slop is kept with nothing
    // to correct, so the rows stay those of the first pass and its factor serves the last.
    for (u32 c = 0; c < mAbsorbed.size(); ++c)
    {
        if (mAbsorbedGroup[c] != static_cast<u32>(&group - mGroups.data()))
            continue;
        const Contact& contact = *mAbsorbed[c];
        const ContactManifold& manifold = contact.manifold;
        for (u32 k = 0; k < manifold.count; ++k)
        {
            const ContactPoint& point = manifold.points[k];
            const Math::vec3 pointA = contact.a->pointToWorld(point.localA);
            const Math::vec3 pointB = contact.b->pointToWorld(point.localB);
            const f32 separation =
                Math::max(Math::dot(pointB - pointA, manifold.normal) + mSlop, -mMaxPenetration);
            if (separation >= mSlop)
                continue;
            const Math::vec3 middle = (pointA + pointB) * 0.5f;
            Row row;
            makeRow(row, contact.a, contact.b,
                    detail::linearRow(middle - contact.a->position(), middle - contact.b->position(), manifold.normal));
            row.error = mBaumgarte * Math::min(separation, 0.0f);
            largest = Math::max(largest, fabsf(row.error));
            mPositionRows.push_back(row);
        }
    }
    u32 n = static_cast<u32>(mPositionRows.size());
    if (n == 0 || largest == 0.0f)
        return;

    // One Newton step on all rows: K dlambda = -C. The first position pass
    // factorises K at its pose; later passes reuse that factor with their own
    // rows and errors (quasi-Newton) when the rows are the factorised ones
    // (same bodies, Jacobian within 2%).
    bool reuse = false;
    if (group.positionFactored)
    {
        // This pass's rows against the factorised ones, in order: a row gone
        // since (limit no longer passed, servo on target) stays in, held (no
        // error); a new one forces a fresh factorisation.
        const Row* factored = mPositionFactorRows.data() + group.positionDiagonal;
        const u32 count = group.positionRows;
        mMergedRows.clear();
        u32 matched = 0;
        for (u32 f = 0; f < count; ++f)
        {
            if (matched < n && sameRow(mPositionRows[matched], factored[f]))
            {
                mMergedRows.push_back(mPositionRows[matched++]);
                continue;
            }
            Row held = factored[f];
            held.error = 0.0f;
            mMergedRows.push_back(held);
        }
        if (matched == n)
        {
            reuse = true;
            n = count;
            mPositionRows.resize(n);
            for (u32 i = 0; i < n; ++i)
                mPositionRows[i] = mMergedRows[i];
        }
    }
    if (!reuse)
    {
        group.positionMatrix = static_cast<u32>(mPositionFactors.size());
        mPositionFactors.resize(mPositionFactors.size() + n * n);
        group.positionDiagonal = static_cast<u32>(mPositionFactorDiagonal.size());
        mPositionFactorDiagonal.resize(mPositionFactorDiagonal.size() + n);
        mPositionFactorDropped.resize(mPositionFactorDiagonal.size());
        mPositionFactorFirst.resize(mPositionFactorDiagonal.size());
        mPositionFactorRows.resize(mPositionFactorDiagonal.size());
        for (u32 i = 0; i < n; ++i)
            mPositionFactorRows[group.positionDiagonal + i] = mPositionRows[i];
        buildMatrix(mPositionRows.data(), n, mPositionFactors.data() + group.positionMatrix);
        factor(mPositionFactors.data() + group.positionMatrix, n, mPositionFactorDiagonal.data() + group.positionDiagonal,
               mPositionFactorDropped.data() + group.positionDiagonal,
               mPositionFactorFirst.data() + group.positionDiagonal);
        group.positionFactored = true;
        group.positionRows = n;
    }
    mWork.resize(n);
    for (u32 i = 0; i < n; ++i)
        mWork[i] = -static_cast<f64>(mPositionRows[i].error);
    solveFactored(mPositionFactors.data() + group.positionMatrix, n,
                  mPositionFactorDiagonal.data() + group.positionDiagonal,
                  mPositionFactorDropped.data() + group.positionDiagonal,
                  mPositionFactorFirst.data() + group.positionDiagonal, mWork.data());
    const f32 scale = largest > kMaxCorrection ? kMaxCorrection / largest : 1.0f;

    mPositionDelta.resize(2 * group.bodyCount);
    for (u32 b = 0; b < 2 * group.bodyCount; ++b)
        mPositionDelta[b] = Math::vec3(0.0f);
    for (u32 i = 0; i < n; ++i)
    {
        const f32 lambda = static_cast<f32>(mWork[i]) * scale;
        if (lambda == 0.0f || !isfinite(lambda))
            continue;
        const Row& row = mPositionRows[i];
        for (u32 k = 0; k < row.parts; ++k)
        {
            mPositionDelta[2 * row.slot[k]] += row.wLinear[k] * lambda;
            mPositionDelta[2 * row.slot[k] + 1] += row.wAngular[k] * lambda;
        }
    }
    for (u32 b = 0; b < group.bodyCount; ++b)
    {
        RigidBody* body = mGroupBodies[group.firstBody + b];
        const Math::vec3& linear = mPositionDelta[2 * b];
        const Math::vec3& angular = mPositionDelta[2 * b + 1];
        if (linear == Math::vec3(0.0f) && angular == Math::vec3(0.0f))
            continue;
        body->setPosition(body->position() + linear);
        const Math::quat spin(0.0f, angular);
        body->setOrientation(body->orientation() + 0.5f * spin * body->orientation());
        body->calculateDerivedData();
    }
}

void ArticulationSolver::solvePosition(u32 pass, u32 passes)
{
    if (pass != 0 && pass + 1 != passes)
        return;
    for (Group& group : mGroups)
        if (group.solved)
            positionGroup(group);
}

} // namespace kinetix::Physics
