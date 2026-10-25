#ifndef KINETIX_PHYSICS_DYNAMICS_SOFTSPRING_H
#define KINETIX_PHYSICS_DYNAMICS_SOFTSPRING_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// One constraint row turned into a spring and solved implicitly inside the
// solver. An external -k*x - c*v is integrated explicitly and only stable while
// k*dt^2/m stays small; the implicit row is unconditionally stable and costs
// nothing extra. The implicit integration adds damping of its own, so even zero
// damping does not oscillate forever.
// Usage, per row, per step:
//   setup:   calculate(...) -> effective mass for the row
//   solve:   lambda = -effectiveMass * (Jv + bias(totalLambda))
class SoftSpring
{
public:
    // `positionError` is the constraint C, how far the row is from where the
    // spring wants it. `stiffness` is k in N/m (or N*m/rad), `damping` c in
    // F = -k*x - c*v. A stiffness of zero leaves the row hard and ignores `damping`.
    //
    // `inverseEffectiveMass` is J*M^-1*J^T for the row; `bias` is added to, not
    // replaced.
    void calculate(f32 duration, f32 inverseEffectiveMass, f32 bias, f32 positionError,
                   f32 stiffness, f32 damping, f32& outEffectiveMass);

    void calculateHard(f32 inverseEffectiveMass, f32 bias, f32& outEffectiveMass);

    bool active() const
    {
        return mSoftness != 0.0f;
    }

    // The full bias given the impulse accumulated so far: the softness term makes
    // the row yield instead of holding.
    f32 bias(f32 totalImpulse) const
    {
        return mSoftness * totalImpulse + mBias;
    }

private:
    f32 mBias = 0.0f;
    f32 mSoftness = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_SOFTSPRING_H
