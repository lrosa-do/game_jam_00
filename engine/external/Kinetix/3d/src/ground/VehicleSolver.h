#ifndef KINETIX_PHYSICS_GROUND_VEHICLESOLVER_H
#define KINETIX_PHYSICS_GROUND_VEHICLESOLVER_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

// Internal to the ground module: one step of a vehicle's velocity-level implicit solve.
// Unknowns are impulses on "rows", each a linear relation between generalised velocities
// (chassis 6 DOF plus scalar shafts). A row's relative velocity is u_i = J_i . v - offset_i;
// an impulse lambda_i changes v by M^-1 J_i^T lambda_i, so u = u_free + W lambda with
// W = J M^-1 J^T.
//  - Rigid: u_i = target_i, lambda_i in [lo_i, hi_i] (gear mesh and differential kinematics
//    unbounded; friction elements target 0, bounds +-capacity; unilateral contact lo = 0).
//  - Soft: lambda_i = A_i - B_i u_i, lambda_i in [lo_i, hi_i]: a spring or damper
//    linearised at the end of the step (backward Euler), stable at any step size. B = 0 is
//    a known (explicit) impulse.
// Two rows may be tied by a friction ellipse (lambda_i / c_i)^2 + (lambda_j / c_j)^2 <= 1
// (the tyre's two directions).
//
// Both kinds are the optimality conditions of one strictly convex QP,
//     minimise 1/2 lambda^T (W + D) lambda - b^T lambda,  lo <= lambda <= hi,
// D = 1/B on soft rows (a tiny regularisation on rigid ones so redundant constraints stay
// solvable), b = A/B - u_free on soft rows, target - u_free on rigid ones. Solved by a
// primal active-set method: lambda stays feasible; each iteration moves to the minimiser of
// the free rows (direct Cholesky in double: a 300:1 gear chain between a heavy wheel and a
// light flywheel stays exact) or stops at the first bound and holds it; a held bound whose
// gradient wants to leave is released one at a time. An unfinished solve is still within
// every bound.
//
// Ellipses are handled outside that loop: a pair outside its ellipse is boxed by
// |lambda_i| <= c_i |d_i|, |lambda_j| <= c_j |d_j| for the unit direction d (ellipse
// coordinates) of its wanted impulse (the box corner sits on the ellipse) and re-solved a
// few times, turning d towards the wanted impulse and releasing pairs that come back inside.
struct SolverRow
{
    enum class Kind : u8
    {
        Rigid,
        Soft
    };

    static constexpr u32 MaxDofs = 3;

    Kind kind = Kind::Rigid;
    // Chassis part of the Jacobian, world space: linear and angular (about the centre of mass).
    Math::vec3 linear{0.0f};
    Math::vec3 angular{0.0f};
    u32 dof[MaxDofs] = {0, 0, 0};
    f32 coefficient[MaxDofs] = {0.0f, 0.0f, 0.0f};
    u32 dofCount = 0;

    f32 offset = 0.0f;
    f32 target = 0.0f;
    f32 softA = 0.0f;
    f32 softB = 0.0f;
    f32 lo = -1.0e30f;
    f32 hi = 1.0e30f;
    // Friction ellipse partner (a row index) and this row's semi-axis.
    s32 partner = -1;
    f32 ellipseCap = 0.0f;

    // Identifies the row across steps for warm starting; 0 for none.
    u32 key = 0;

    f32 impulse = 0.0f;
    f32 velocity = 0.0f;

    void addDof(u32 index, f32 value)
    {
        if (dofCount < MaxDofs)
        {
            dof[dofCount] = index;
            coefficient[dofCount] = value;
            ++dofCount;
        }
    }
};

class VehicleSolver
{
public:
    void reset(u32 dofCount);
    // Free velocities: what the chassis and shafts would do this step with only gravity and the
    // explicit forces acting.
    void setChassis(f32 inverseMass, const Math::mat3& inverseInertiaWorld, const Math::vec3& velocity,
                    const Math::vec3& angularVelocity);
    void setDof(u32 index, f32 inverseInertia, f32 velocity);

    u32 addRow(const SolverRow& row);
    SolverRow& row(u32 index)
    {
        return mRows[index];
    }
    u32 rowCount() const
    {
        return static_cast<u32>(mRows.size());
    }

    // Solves for the impulses; afterwards every row has its impulse and the
    // relative velocity it ends the step with, and the velocities below are
    // the step's end velocities. With warmStart the rows' impulses (from the
    // last solve, or set by the caller; clamped into their bounds) are the
    // starting point. With reuseMatrix the rows' Jacobians, offsets and the
    // free velocities are those of the last solve and only their soft terms,
    // targets and bounds may have changed: the effective-mass matrix is kept.
    void solve(bool warmStart = false, bool reuseMatrix = false);

    const Math::vec3& chassisVelocity() const
    {
        return mVelocity;
    }
    const Math::vec3& chassisAngularVelocity() const
    {
        return mAngularVelocity;
    }
    // Total impulse on the chassis: linear, and angular about the centre of mass.
    const Math::vec3& chassisLinearImpulse() const
    {
        return mLinearImpulse;
    }
    const Math::vec3& chassisAngularImpulse() const
    {
        return mAngularImpulse;
    }
    f32 dofVelocity(u32 index) const
    {
        return mDofVelocity[index];
    }
    // Active-set iterations the last solve took (all ellipse rounds).
    u32 iterations() const
    {
        return mIterations;
    }
    bool converged() const
    {
        return mConverged;
    }

private:
    void buildMatrix();
    bool factorAndSolve(u32 size);
    // Box-constrained QP over the rows not held explicit, from the current (feasible) mLambda.
    bool solveBox();
    f64 gradient(u32 row) const;

    ct::Vector<SolverRow> mRows;
    f32 mInverseMass = 0.0f;
    Math::mat3 mInverseInertia{0.0f};
    Math::vec3 mFreeVelocity{0.0f};
    Math::vec3 mFreeAngularVelocity{0.0f};
    ct::Vector<f32> mDofInverseInertia;
    ct::Vector<f32> mDofFreeVelocity;
    ct::Vector<f32> mDofVelocity;

    Math::vec3 mVelocity{0.0f};
    Math::vec3 mAngularVelocity{0.0f};
    Math::vec3 mLinearImpulse{0.0f};
    Math::vec3 mAngularImpulse{0.0f};
    u32 mIterations = 0;
    bool mConverged = true;

    ct::Vector<f64> mW;      // n x n effective mass
    ct::Vector<f64> mA;      // W + D
    ct::Vector<f64> mB;      // linear term
    ct::Vector<f64> mFreeU;  // relative velocity with no impulses
    ct::Vector<f64> mLambda;
    ct::Vector<f64> mLo;     // bounds in force for this round
    ct::Vector<f64> mHi;
    ct::Vector<s8> mState;   // 0 free, -1 held at lo, +1 held at hi, 2 explicit
    ct::Vector<u8> mBoxed;   // ellipse pair currently boxed
    ct::Vector<f64> mDirection; // its direction, in ellipse coordinates
    ct::Vector<u32> mFree;
    ct::Vector<f64> mMatrix; // reduced system, then its Cholesky factor
    ct::Vector<f64> mRhs;
    ct::Vector<f64> mCandidate;
    ct::Vector<Math::vec3> mMinvLinear;
    ct::Vector<Math::vec3> mMinvAngular;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_VEHICLESOLVER_H
