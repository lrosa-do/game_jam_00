#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "VehicleSolver.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
// Relative compliance on rigid rows: small enough to leave no measurable error, large enough
// that redundant rows still factorise.
constexpr f64 kRigidRegularisation = 1.0e-9;
constexpr u32 kEllipseRounds = 4;
} // namespace

void VehicleSolver::reset(u32 dofCount)
{
    mRows.clear();
    mDofInverseInertia.resize(dofCount);
    mDofFreeVelocity.resize(dofCount);
    mDofVelocity.resize(dofCount);
    for (u32 i = 0; i < dofCount; ++i)
    {
        mDofInverseInertia[i] = 0.0f;
        mDofFreeVelocity[i] = 0.0f;
        mDofVelocity[i] = 0.0f;
    }
    mInverseMass = 0.0f;
    mInverseInertia = Math::mat3(0.0f);
    mFreeVelocity = Math::vec3(0.0f);
    mFreeAngularVelocity = Math::vec3(0.0f);
    mLinearImpulse = Math::vec3(0.0f);
    mAngularImpulse = Math::vec3(0.0f);
    mIterations = 0;
    mConverged = true;
}

void VehicleSolver::setChassis(f32 inverseMass, const Math::mat3& inverseInertiaWorld,
                               const Math::vec3& velocity, const Math::vec3& angularVelocity)
{
    mInverseMass = inverseMass;
    mInverseInertia = inverseInertiaWorld;
    mFreeVelocity = velocity;
    mFreeAngularVelocity = angularVelocity;
}

void VehicleSolver::setDof(u32 index, f32 inverseInertia, f32 velocity)
{
    if (index >= mDofInverseInertia.size())
        return;
    mDofInverseInertia[index] = inverseInertia;
    mDofFreeVelocity[index] = velocity;
}

u32 VehicleSolver::addRow(const SolverRow& row)
{
    mRows.push_back(row);
    return static_cast<u32>(mRows.size() - 1);
}

bool VehicleSolver::factorAndSolve(u32 size)
{
    // In-place Cholesky of mMatrix, then substitution of mRhs. The matrix is W on the free rows
    // plus a positive diagonal; a pivot collapsed by rounding on a near-redundant set is floored
    // instead of producing a NaN.
    f64* a = mMatrix.data();
    for (u32 j = 0; j < size; ++j)
    {
        f64* rowJ = a + static_cast<usize>(j) * size;
        f64 diagonal = rowJ[j];
        for (u32 k = 0; k < j; ++k)
            diagonal -= rowJ[k] * rowJ[k];
        const f64 floor = 1.0e-14 * (1.0 + fabs(rowJ[j]));
        if (!(diagonal > floor))
            diagonal = floor;
        const f64 root = sqrt(diagonal);
        const f64 inverseRoot = 1.0 / root;
        rowJ[j] = root;
        for (u32 i = j + 1; i < size; ++i)
        {
            f64* rowI = a + static_cast<usize>(i) * size;
            f64 value = rowI[j];
            for (u32 k = 0; k < j; ++k)
                value -= rowI[k] * rowJ[k];
            rowI[j] = value * inverseRoot;
        }
    }
    f64* b = mRhs.data();
    for (u32 i = 0; i < size; ++i)
    {
        const f64* rowI = a + static_cast<usize>(i) * size;
        f64 value = b[i];
        for (u32 k = 0; k < i; ++k)
            value -= rowI[k] * b[k];
        b[i] = value / rowI[i];
    }
    for (u32 ii = size; ii-- > 0;)
    {
        f64 value = b[ii];
        for (u32 k = ii + 1; k < size; ++k)
            value -= a[static_cast<usize>(k) * size + ii] * b[k];
        b[ii] = value / a[static_cast<usize>(ii) * size + ii];
    }
    for (u32 i = 0; i < size; ++i)
        if (!isfinite(b[i]))
            return false;
    return true;
}

f64 VehicleSolver::gradient(u32 row) const
{
    const u32 n = static_cast<u32>(mRows.size());
    const f64* a = &mA[static_cast<usize>(row) * n];
    f64 g = -mB[row];
    for (u32 j = 0; j < n; ++j)
        g += a[j] * mLambda[j];
    return g;
}

bool VehicleSolver::solveBox()
{
    const u32 n = static_cast<u32>(mRows.size());
    for (u32 i = 0; i < n; ++i)
    {
        if (mState[i] == 2)
            continue;
        mLambda[i] = Clamp(mLambda[i], mLo[i], mHi[i]);
        if (mLambda[i] <= mLo[i] && mLo[i] > -1.0e29)
            mState[i] = -1;
        else if (mLambda[i] >= mHi[i] && mHi[i] < 1.0e29)
            mState[i] = 1;
        else
            mState[i] = 0;
    }

    // Raw pointers: these loops run thousands of times a step and checked accessors dominate in
    // an unoptimised build.
    const f64* A = mA.data();
    const f64* B = mB.data();
    const f64* lo = mLo.data();
    const f64* hi = mHi.data();
    f64* lambda = mLambda.data();
    s8* state = mState.data();
    mFree.resize(n);
    mMatrix.resize(static_cast<usize>(n) * n);
    mRhs.resize(n);
    u32* freeRows = mFree.data();

    const u32 maxIterations = 4 * n + 16;
    for (u32 iteration = 0; iteration < maxIterations; ++iteration)
    {
        ++mIterations;
        u32 m = 0;
        for (u32 i = 0; i < n; ++i)
            if (state[i] == 0)
                freeRows[m++] = i;

        bool reached = true;
        if (m > 0)
        {
            f64* matrix = mMatrix.data();
            f64* rhs = mRhs.data();
            for (u32 a = 0; a < m; ++a)
            {
                const u32 i = freeRows[a];
                const f64* ai = A + static_cast<usize>(i) * n;
                f64* row = matrix + static_cast<usize>(a) * m;
                for (u32 b = 0; b < m; ++b)
                    row[b] = ai[freeRows[b]];
                f64 value = B[i];
                for (u32 j = 0; j < n; ++j)
                    if (state[j] != 0)
                        value -= ai[j] * lambda[j];
                rhs[a] = value;
            }
            if (!factorAndSolve(m))
                return false;

            f64 step = 1.0;
            s32 blocking = -1;
            s8 side = 0;
            for (u32 a = 0; a < m; ++a)
            {
                const u32 i = freeRows[a];
                const f64 target = rhs[a];
                const f64 delta = target - lambda[i];
                if (target < lo[i] && delta < 0.0)
                {
                    const f64 t = (lo[i] - lambda[i]) / delta;
                    if (t < step)
                    {
                        step = t;
                        blocking = static_cast<s32>(i);
                        side = -1;
                    }
                }
                else if (target > hi[i] && delta > 0.0)
                {
                    const f64 t = (hi[i] - lambda[i]) / delta;
                    if (t < step)
                    {
                        step = t;
                        blocking = static_cast<s32>(i);
                        side = 1;
                    }
                }
            }
            step = step < 0.0 ? 0.0 : (step > 1.0 ? 1.0 : step);
            for (u32 a = 0; a < m; ++a)
            {
                const u32 i = freeRows[a];
                lambda[i] += step * (rhs[a] - lambda[i]);
            }
            if (blocking >= 0)
            {
                const u32 k = static_cast<u32>(blocking);
                lambda[k] = side < 0 ? lo[k] : hi[k];
                state[k] = side;
                reached = false;
            }
        }
        if (!reached)
            continue;

        f64 worst = 0.0;
        s32 release = -1;
        for (u32 i = 0; i < n; ++i)
        {
            if (state[i] != -1 && state[i] != 1)
                continue;
            const f64* ai = A + static_cast<usize>(i) * n;
            f64 g = -B[i];
            for (u32 j = 0; j < n; ++j)
                g += ai[j] * lambda[j];
            const f64 diagonal = ai[i] > 1.0e-30 ? ai[i] : 1.0e-30;
            g /= diagonal;
            // Held at lo: optimal while g >= 0; at hi while g <= 0.
            const f64 want = state[i] == -1 ? -g : g;
            const f64 tolerance = 1.0e-7 * (1.0 + fabs(lambda[i]));
            if (want > tolerance && want > worst)
            {
                worst = want;
                release = static_cast<s32>(i);
            }
        }
        if (release < 0)
            return true;
        state[static_cast<u32>(release)] = 0;
    }
    return false;
}

void VehicleSolver::buildMatrix()
{
    const u32 n = static_cast<u32>(mRows.size());
    const u32 dofs = static_cast<u32>(mDofInverseInertia.size());
    mMinvLinear.resize(n);
    mMinvAngular.resize(n);
    mW.resize(static_cast<usize>(n) * n);
    mFreeU.resize(n);
    f64* W = mW.data();
    for (u32 i = 0; i < n; ++i)
    {
        const SolverRow& r = mRows[i];
        mMinvLinear[i] = r.linear * mInverseMass;
        mMinvAngular[i] = mInverseInertia * r.angular;
        f64 u = static_cast<f64>(Math::dot(r.linear, mFreeVelocity)) +
                static_cast<f64>(Math::dot(r.angular, mFreeAngularVelocity));
        for (u32 k = 0; k < r.dofCount; ++k)
            if (r.dof[k] < dofs)
                u += static_cast<f64>(r.coefficient[k]) * mDofFreeVelocity[r.dof[k]];
        mFreeU[i] = u - r.offset;
    }
    for (u32 i = 0; i < n; ++i)
    {
        const SolverRow& ri = mRows[i];
        for (u32 j = i; j < n; ++j)
        {
            const SolverRow& rj = mRows[j];
            f64 w = static_cast<f64>(Math::dot(ri.linear, mMinvLinear[j])) +
                    static_cast<f64>(Math::dot(ri.angular, mMinvAngular[j]));
            for (u32 a = 0; a < ri.dofCount; ++a)
            {
                for (u32 b = 0; b < rj.dofCount; ++b)
                {
                    if (ri.dof[a] == rj.dof[b] && ri.dof[a] < dofs)
                    {
                        w += static_cast<f64>(ri.coefficient[a]) * rj.coefficient[b] *
                             mDofInverseInertia[ri.dof[a]];
                    }
                }
            }
            W[static_cast<usize>(i) * n + j] = w;
            W[static_cast<usize>(j) * n + i] = w;
        }
    }
}

void VehicleSolver::solve(bool warmStart, bool reuseMatrix)
{
    const u32 n = static_cast<u32>(mRows.size());
    const u32 dofs = static_cast<u32>(mDofInverseInertia.size());
    mIterations = 0;
    mConverged = true;
    if (reuseMatrix && mW.size() != static_cast<usize>(n) * n)
        reuseMatrix = false;

    if (!reuseMatrix)
        buildMatrix();
    mA.resize(static_cast<usize>(n) * n);
    mB.resize(n);
    mLambda.resize(n);
    mLo.resize(n);
    mHi.resize(n);
    mState.resize(n);
    mBoxed.resize(n);
    mDirection.resize(n);
    {
        const f64* W = mW.data();
        f64* A = mA.data();
        for (usize k = 0; k < mW.size(); ++k)
            A[k] = W[k];
    }
    for (u32 i = 0; i < n; ++i)
    {
        const SolverRow& r = mRows[i];
        f64& diagonal = mA[static_cast<usize>(i) * n + i];
        mBoxed[i] = 0;
        mDirection[i] = 0.0;
        mLo[i] = Min(static_cast<f64>(r.lo), static_cast<f64>(r.hi));
        mHi[i] = static_cast<f64>(r.hi);
        mState[i] = 0;
        mLambda[i] = warmStart && isfinite(r.impulse) ? static_cast<f64>(r.impulse) : 0.0;
        if (r.kind == SolverRow::Kind::Soft)
        {
            if (!(r.softB > 0.0f))
            {
                mState[i] = 2;
                mLambda[i] = Clamp(static_cast<f64>(r.softA), mLo[i], mHi[i]);
                mB[i] = 0.0;
                continue;
            }
            // lambda = A - B (u_free + W lambda): stationary point of
            // 1/2 lambda (W + 1/B) lambda - (A/B - u_free) lambda.
            const f64 inverseB = 1.0 / static_cast<f64>(r.softB);
            diagonal += inverseB;
            mB[i] = static_cast<f64>(r.softA) * inverseB - mFreeU[i];
        }
        else
        {
            diagonal += kRigidRegularisation * (diagonal + 1.0e-12);
            mB[i] = static_cast<f64>(r.target) - mFreeU[i];
        }
        if (r.partner >= 0 && static_cast<u32>(r.partner) < n)
        {
            const f64 cap = Max(static_cast<f64>(r.ellipseCap), 0.0);
            mLo[i] = Max(mLo[i], -cap);
            mHi[i] = Min(mHi[i], cap);
        }
    }

    for (u32 round = 0; round < kEllipseRounds; ++round)
    {
        if (!solveBox())
            mConverged = false;
        bool changed = false;
        for (u32 i = 0; i < n; ++i)
        {
            const SolverRow& r = mRows[i];
            if (r.partner < 0 || static_cast<u32>(r.partner) <= i || static_cast<u32>(r.partner) >= n)
                continue;
            const u32 j = static_cast<u32>(r.partner);
            if (mState[i] == 2 || mState[j] == 2)
                continue;
            const f64 ci = Max(static_cast<f64>(r.ellipseCap), 1.0e-20);
            const f64 cj = Max(static_cast<f64>(mRows[j].ellipseCap), 1.0e-20);
            // Where the pair wants to be: its rows' own optimum with the rest as it is (lambda - g / a_ii).
            const f64 aii = Max(mA[static_cast<usize>(i) * n + i], 1.0e-30);
            const f64 ajj = Max(mA[static_cast<usize>(j) * n + j], 1.0e-30);
            const f64 wantI = mLambda[i] - gradient(i) / aii;
            const f64 wantJ = mLambda[j] - gradient(j) / ajj;
            const f64 xi = wantI / ci;
            const f64 xj = wantJ / cj;
            const f64 e = xi * xi + xj * xj;
            if (!mBoxed[i])
            {
                const f64 li = mLambda[i] / ci;
                const f64 lj = mLambda[j] / cj;
                if (li * li + lj * lj <= 1.0 + 1.0e-9)
                    continue;
                const f64 length = sqrt(li * li + lj * lj);
                mBoxed[i] = 1;
                mDirection[i] = li / length;
                mDirection[j] = lj / length;
                changed = true;
            }
            else if (e <= 1.0 - 1.0e-6)
            {
                mBoxed[i] = 0;
                mLo[i] = Max(static_cast<f64>(r.lo), -ci);
                mHi[i] = Min(static_cast<f64>(r.hi), ci);
                mLo[j] = Max(static_cast<f64>(mRows[j].lo), -cj);
                mHi[j] = Min(static_cast<f64>(mRows[j].hi), cj);
                changed = true;
                continue;
            }
            else
            {
                // Turn half way to the wanted direction: it moves with the impulse, and a stiff tyre would
                // swing between two.
                const f64 length = sqrt(e);
                f64 di = 0.5 * (mDirection[i] + xi / length);
                f64 dj = 0.5 * (mDirection[j] + xj / length);
                const f64 dl = sqrt(di * di + dj * dj);
                if (dl < 1.0e-12)
                    continue;
                di /= dl;
                dj /= dl;
                if (fabs(di - mDirection[i]) + fabs(dj - mDirection[j]) < 1.0e-3)
                    continue;
                mDirection[i] = di;
                mDirection[j] = dj;
                changed = true;
            }
            const f64 bi = ci * fabs(mDirection[i]);
            const f64 bj = cj * fabs(mDirection[j]);
            mLo[i] = Max(static_cast<f64>(r.lo), -bi);
            mHi[i] = Min(static_cast<f64>(r.hi), bi);
            mLo[j] = Max(static_cast<f64>(mRows[j].lo), -bj);
            mHi[j] = Min(static_cast<f64>(mRows[j].hi), bj);
        }
        if (!changed)
            break;
        if (round + 1 == kEllipseRounds)
        {
            // Out of rounds with a box just moved: solve once more so the impulses honour it.
            if (!solveBox())
                mConverged = false;
        }
    }

    mLinearImpulse = Math::vec3(0.0f);
    mAngularImpulse = Math::vec3(0.0f);
    Math::vec3 dv(0.0f);
    Math::vec3 dw(0.0f);
    for (u32 i = 0; i < dofs; ++i)
        mDofVelocity[i] = mDofFreeVelocity[i];
    for (u32 i = 0; i < n; ++i)
    {
        const SolverRow& r = mRows[i];
        const f32 lambda = isfinite(mLambda[i]) ? static_cast<f32>(mLambda[i]) : 0.0f;
        mLinearImpulse += r.linear * lambda;
        mAngularImpulse += r.angular * lambda;
        dv += mMinvLinear[i] * lambda;
        dw += mMinvAngular[i] * lambda;
        for (u32 k = 0; k < r.dofCount; ++k)
            if (r.dof[k] < dofs)
                mDofVelocity[r.dof[k]] += r.coefficient[k] * lambda * mDofInverseInertia[r.dof[k]];
    }
    mVelocity = mFreeVelocity + dv;
    mAngularVelocity = mFreeAngularVelocity + dw;
    for (u32 i = 0; i < n; ++i)
    {
        SolverRow& r = mRows[i];
        r.impulse = isfinite(mLambda[i]) ? static_cast<f32>(mLambda[i]) : 0.0f;
        f64 u = mFreeU[i];
        const f64* w = &mW[static_cast<usize>(i) * n];
        for (u32 j = 0; j < n; ++j)
            u += w[j] * mLambda[j];
        r.velocity = static_cast<f32>(u);
    }
}

} // namespace kinetix::Physics
