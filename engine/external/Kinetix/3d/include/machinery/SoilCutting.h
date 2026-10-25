#ifndef KINETIX_PHYSICS_MACHINERY_SOILCUTTING_H
#define KINETIX_PHYSICS_MACHINERY_SOILCUTTING_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Joint.h"
#include "machinery/Terramechanics.h"

namespace kinetix::Physics
{

class RigidBody;

// A bed of diggable soil: a level surface at `surfaceHeight` (world y) over [minX, maxX] x
// [minZ, maxZ]. Below the surface a cutting tool meets resistance; the bed is not a rigid body to
// the tool (give the bed's collision body a filter the tool does not collide with).
struct SoilBed
{
    f32 minX = -1.0e9f;
    f32 maxX = 1.0e9f;
    f32 minZ = -1.0e9f;
    f32 maxZ = 1.0e9f;
    f32 surfaceHeight = 0.0f;
    SoilParameters soil;

    // Depth (m, >= 0) of a world point below the surface, 0 outside.
    f32 depth(const Math::vec3& point) const;
};

// Resistance of soil to a cutting edge (bucket teeth, blade edge), as a bounded constraint on the
// edge's motion.
//
// A wide tool of width w at rake angle alpha (from the horizontal) cutting at depth d meets the
// fundamental earthmoving equation in McKyes' form,
//   F = w (gamma d^2 N_gamma + c d N_c) + w rho v^2 d,
//   N_gamma = (cot alpha + cot beta) / (2 D),
//   N_c = (1 + cot beta cot(beta + phi)) / D,
//   D = cos(alpha + delta) + sin(alpha + delta) cot(beta + phi),
// the soil failing along the plane at beta to the horizontal that minimises F (found
// numerically), delta the soil-on-steel friction angle, no surcharge or adhesion. The last term
// is the soil accelerated to the tool's speed v. For a vertical smooth tool it equals Rankine's
// passive pressure (N_gamma = Kp / 2, N_c = 2 sqrt(Kp)).
//
// F acts as a Coulomb element at the middle of the edge: the tool's velocity there is driven to
// zero with at most F (impulse F h) in any direction, so a tool pushed with less than F stalls,
// and with more it cuts through. Isotropic: pulling the tool out is resisted alike (breakout).
// Not modelled: the cut soil leaving the bed, the load in the bucket.
class SoilCuttingTool final : public Joint
{
public:
    // The edge, between two points on the tool body (world, at construction).
    SoilCuttingTool(RigidBody& tool, const Math::vec3& worldEdgeA, const Math::vec3& worldEdgeB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    bool singleBody() const override
    {
        return true;
    }
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setSoilBed(const SoilBed& bed);
    const SoilBed& soilBed() const
    {
        return mBed;
    }
    // Scales the resistance (1 by default; teeth help: < 1).
    void setResistanceScale(f32 scale);
    // Rake angle from the horizontal, rad (45 deg by default).
    void setRakeAngle(f32 radians);

    // From the last step: the edge's mean depth (m), the resistance available (N, the bound) and
    // the force the soil actually applied (world, N).
    f32 depth() const
    {
        return mDepth;
    }
    f32 resistance() const
    {
        return mResistance;
    }
    Math::vec3 force() const;
    Math::vec3 edgeMiddle() const;
    // Cutting resistance, N, for a width and depth at a speed and rake angle (failure plane minimising it).
    static f32 cuttingResistance(const SoilParameters& soil, f32 width, f32 depth, f32 speed, f32 rakeAngle);

private:
    RigidBody* mTool;
    Math::vec3 mLocalA;
    Math::vec3 mLocalB;
    SoilBed mBed;
    f32 mScale = 1.0f;
    f32 mRake = 0.785398163f;
    f32 mDepth = 0.0f;
    f32 mResistance = 0.0f;
    bool mActive = false;
    Math::vec3 mArm{0.0f};
    Math::mat3 mEffectiveMass{0.0f};
    Math::vec3 mTotalImpulse{0.0f};
    f32 mCap = 0.0f;
    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_SOILCUTTING_H
