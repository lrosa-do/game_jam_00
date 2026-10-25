#ifndef KINETIX_PHYSICS_DYNAMICS_JOINTAXIS_H
#define KINETIX_PHYSICS_DYNAMICS_JOINTAXIS_H

// Axis normalisation, effective-mass checks and drive/lock coupling shared by
// the joint implementations. Internal, not a public header.

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Joint.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics::detail
{

// A zero axis would normalize to NaN, which spreads through the impulses into
// every body it touches; fall back to a sane axis so the mistake stays in one joint.
inline Math::vec3 normalizedAxisOr(const Math::vec3& axis, const Math::vec3& fallback)
{
    const f32 length = Math::length(axis);
    if (length > 1.0e-6f && isfinite(length))
        return axis / length;
    return fallback;
}

// Whether an N x N effective-mass matrix (N = 2 or 3) is invertible, judged
// against its own scale: a determinant scales with the N-th power of the
// entries, so a fixed threshold calls heavy bodies singular (a 20 t trailer on
// a point joint has a determinant of ~1e-13). The threshold is 1e-9 for
// unit-sized entries.
inline bool invertible(const Math::mat3& m, f32 determinant)
{
    if (!isfinite(determinant))
        return false;
    f32 scale = 0.0f;
    for (u32 c = 0; c < 3; ++c)
        for (u32 r = 0; r < 3; ++r)
            scale = Math::max(scale, fabsf(m[c][r]));
    return scale > 0.0f && fabsf(determinant) > 1.0e-9f * scale * scale * scale;
}

inline bool invertible(const Math::mat2& m, f32 determinant)
{
    if (!isfinite(determinant))
        return false;
    f32 scale = 0.0f;
    for (u32 c = 0; c < 2; ++c)
        for (u32 r = 0; r < 2; ++r)
            scale = Math::max(scale, fabsf(m[c][r]));
    return scale > 0.0f && fabsf(determinant) > 1.0e-9f * scale * scale;
}

// How an angular impulse `axis` (on B, opposite on A) changes the relative
// velocity of two anchor points: what a point lock feels from an angular row.
inline Math::vec3 pointCoupling(const Math::mat3& inverseInertiaA, const Math::mat3& inverseInertiaB,
                                const Math::vec3& armA, const Math::vec3& armB,
                                const Math::vec3& axis)
{
    return Math::cross(inverseInertiaB * axis, armB) + Math::cross(inverseInertiaA * axis, armA);
}

using JacobianRow = JointRow;

// J_p M^-1 J_q^T for two rows on the same pair of bodies.
inline f32 rowProduct(const RigidBody& a, const RigidBody& b, const JacobianRow& p,
                      const JacobianRow& q)
{
    return a.inverseMass() * Math::dot(p.linearA, q.linearA) +
           Math::dot(p.angularA, a.inverseInertiaTensorWorld() * q.angularA) +
           b.inverseMass() * Math::dot(p.linearB, q.linearB) +
           Math::dot(p.angularB, b.inverseInertiaTensorWorld() * q.angularB);
}

// A drive row and its joint's lock rows. The locks hold whatever the drive
// does, so the axis drives the bodies as the locks hold them: the Schur
// complement J_d M^-1 J_d^T - c^T K^-1 c, with K = J_l M^-1 J_l^T and
// c = J_l M^-1 J_d^T (a hinge servo's own row sees the arm's inertia about its
// centre of mass; held at the pivot it is that about the pivot, a quarter of
// it for a uniform rod).
//
// That reduced inverse mass always scales the servo's frequency gains
// (gainInverseMass). How the row is solved depends on the pair:
//  - One side immovable (static or kinematic): solved as one block with the
//    locks. Each unit of drive impulse carries the lock impulses -K^-1 c that
//    leave the locks undisturbed (`compensation`), and the row's inverse mass is
//    the reduced one, so the drive gets what it asks for in one iteration.
//  - Both sides dynamic (a joint inside a chain): solved alone with its bare
//    inverse mass and no compensation. The block's lock impulses push the parent
//    link along its length and, with every joint doing that, the velocity
//    iterations stop converging (rigid servos chatter, torques flipping sign).
//
// At most MaxLockRows lock rows. A singular lock block falls back to the bare
// row with no compensation.
constexpr u32 MaxLockRows = 5;

struct DriveCoupling
{
    // What the drive row is solved with, and what its gains are scaled by.
    f32 inverseMass = 0.0f;
    f32 gainInverseMass = 0.0f;
};

inline DriveCoupling coupleDriveToLocks(const RigidBody& a, const RigidBody& b,
                                        const JacobianRow& drive, const JacobianRow* locks, u32 count,
                                        f32* compensation)
{
    const f32 bare = rowProduct(a, b, drive, drive);
    DriveCoupling coupling;
    coupling.inverseMass = bare;
    coupling.gainInverseMass = bare;
    for (u32 i = 0; i < count; ++i)
        compensation[i] = 0.0f;
    if (count == 0 || count > MaxLockRows || !(bare > 0.0f))
        return coupling;

    // [K | c] by Gaussian elimination with partial pivoting (symmetric positive
    // definite unless singular, at most five rows).
    f32 k[MaxLockRows][MaxLockRows + 1];
    f32 scale = 0.0f;
    for (u32 i = 0; i < count; ++i)
    {
        for (u32 j = 0; j < count; ++j)
        {
            k[i][j] = rowProduct(a, b, locks[i], locks[j]);
            scale = Math::max(scale, fabsf(k[i][j]));
        }
        k[i][count] = rowProduct(a, b, locks[i], drive);
    }
    if (!(scale > 0.0f) || !isfinite(scale))
        return coupling;
    for (u32 column = 0; column < count; ++column)
    {
        u32 pivot = column;
        for (u32 row = column + 1; row < count; ++row)
            if (fabsf(k[row][column]) > fabsf(k[pivot][column]))
                pivot = row;
        if (fabsf(k[pivot][column]) <= 1.0e-7f * scale)
            return coupling;
        if (pivot != column)
            for (u32 j = 0; j <= count; ++j)
            {
                const f32 swap = k[column][j];
                k[column][j] = k[pivot][j];
                k[pivot][j] = swap;
            }
        for (u32 row = column + 1; row < count; ++row)
        {
            const f32 factor = k[row][column] / k[column][column];
            for (u32 j = column; j <= count; ++j)
                k[row][j] -= factor * k[column][j];
        }
    }
    f32 x[MaxLockRows];
    for (u32 row = count; row-- > 0;)
    {
        f32 sum = k[row][count];
        for (u32 j = row + 1; j < count; ++j)
            sum -= k[row][j] * x[j];
        x[row] = sum / k[row][row];
    }

    f32 reduced = bare;
    for (u32 i = 0; i < count; ++i)
        reduced -= rowProduct(a, b, locks[i], drive) * x[i];
    // A drive the locks already determine has no inverse mass left; keep a
    // millionth of the bare row's.
    if (!(reduced > 1.0e-6f * bare) || !isfinite(reduced))
        return coupling;
    coupling.gainInverseMass = reduced;
    if (a.isDynamic() && b.isDynamic())
        return coupling;
    coupling.inverseMass = reduced;
    for (u32 i = 0; i < count; ++i)
        compensation[i] = -x[i];
    return coupling;
}

// A row along `direction` between two anchors (arms from each centre of mass):
// relative velocity of B's anchor to A's along it.
inline JacobianRow linearRow(const Math::vec3& armA, const Math::vec3& armB,
                             const Math::vec3& direction)
{
    JacobianRow row;
    row.linearB = direction;
    row.angularB = Math::cross(armB, direction);
    row.linearA = -direction;
    row.angularA = -Math::cross(armA, direction);
    return row;
}

// A row about `axis`: B's angular velocity relative to A's.
inline JacobianRow angularRow(const Math::vec3& axis)
{
    JacobianRow row;
    row.angularB = axis;
    row.angularA = -axis;
    return row;
}

// Position-pass angular impulse: B turned by I_B^-1 * impulse, A by the opposite.
inline void applyAngularPositionImpulse(RigidBody& a, RigidBody& b, const Math::vec3& impulse)
{
    if (a.isDynamic())
    {
        const Math::quat spin(0.0f, a.inverseInertiaTensorWorld() * -impulse);
        a.setOrientation(a.orientation() + 0.5f * spin * a.orientation());
    }
    if (b.isDynamic())
    {
        const Math::quat spin(0.0f, b.inverseInertiaTensorWorld() * impulse);
        b.setOrientation(b.orientation() + 0.5f * spin * b.orientation());
    }
}

// The rows of a point lock between two anchors (arms from each centre of
// mass): relative velocity of the anchors along each world axis.
inline void pointLockRows(const Math::vec3& armA, const Math::vec3& armB, JacobianRow* rows)
{
    const Math::vec3 axes[] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                               Math::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 i = 0; i < 3; ++i)
        rows[i] = linearRow(armA, armB, axes[i]);
}

} // namespace kinetix::Physics::detail

#endif // KINETIX_PHYSICS_DYNAMICS_JOINTAXIS_H
