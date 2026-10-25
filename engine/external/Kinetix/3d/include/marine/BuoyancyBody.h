#ifndef KINETIX_PHYSICS_MARINE_BUOYANCYBODY_H
#define KINETIX_PHYSICS_MARINE_BUOYANCYBODY_H

#include "marine/HullMesh.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// A hull vertex for waterline clipping: depth below the surface (positive submerged) and
// the water velocity there.
struct WetVertex
{
    Math::vec3 position{0.0f};
    f32 depth = 0.0f;
    Math::vec3 flow{0.0f};
};

struct WetTriangle
{
    WetVertex vertices[3];
};

// Clips one triangle to the part below the water, depth linear between the vertices.
// Writes 0, 1 or 2 triangles to `out` and returns the count. Cut points are interpolated
// exactly on the edges (depth zero, flow interpolated), winding is kept, and a vertex
// exactly on the waterline gives no division by zero or negative-area sliver.
// If the triangle crosses the surface, the cut segment goes to waterline[0..1] (in the
// order of the clipped polygon's boundary) when not null, and `*crosses` is set.
u32 clipTriangleToWater(const WetVertex in[3], WetTriangle out[2], Math::vec3* waterline = nullptr,
                        bool* crosses = nullptr);

// ITTC 1957 correlation line, Cf = 0.075 / (log10 Re - 2)^2: flat plate of the hull's
// length and wetted area, turbulent from the leading edge. Re below 1e5 is raised to 1e5.
f32 ittc57FrictionCoefficient(f32 reynolds);

// Every pressure is gauge pressure.
//
// Pressure drag and lift: each submerged face sees the water at v_r = v_point - u_water
// (u_water = orbital velocity + current), cos(theta) = (v_r . n) / |v_r|, n outward. A face
// moving into the water (cos > 0) carries
//   p = rho (cLin |v_r| + 1/2 cQuad |v_r|^2) cos^f(theta)
// pushing it back (-p A n); moving away (cos < 0) the same form with its own coefficients
// gives a suction (+p A n). The linear term dominates slow motion (and settles a heaving
// body), the quadratic one fast motion. f = 1 makes planing lift linear in trim angle,
// f = 2 is Newtonian impact theory. With ventilation, suction cannot exceed the hydrostatic
// pressure at the face (the water separates; a transom runs dry at speed).
//
// Skin friction along each face against the tangential v_r:
//   tau = 1/2 rho Cf (1 + k) |v_t| v_t,  Cf = ITTC-57 at Re = U L / nu
// U the speed through the water, L the reference length, k the form factor.
//
// Slamming (von Karman / Wagner water entry, F = rho (pi/2) c V dA/dt per unit length):
// faces crossing the surface whose wetted area grew this step get
//   F = slamCoefficient * rho * slamLength * v_n * dA/dt,
// along -n, capped by maxSlamPressure over the wetted area. The default coefficient pi/4 is
// von Karman's pi/2 with the mean wetted half-width over an entry. Needs a step to measure
// dA/dt, so computeForces() leaves it out.
struct HydrodynamicCoefficients
{
    f32 pressureLinear = 0.6f;    // m/s
    f32 pressureQuadratic = 0.9f;
    f32 pressureFalloff = 1.0f;
    f32 suctionLinear = 0.6f;     // m/s
    f32 suctionQuadratic = 0.4f;
    f32 suctionFalloff = 1.0f;
    bool ventilation = true;

    bool skinFriction = true;
    f32 formFactor = 0.1f;        // k in (1 + k)

    f32 slamCoefficient = 0.785f;
    f32 slamLength = 0.0f;        // m; zero takes half the hull's beam
    f32 maxSlamPressure = 2.0e5f; // Pa

    // Explicit integration of a stiff drag can overshoot and reverse the motion it opposes.
    // When on, the dissipative force along the velocity through the water (and the torque along
    // the rotation) is limited to what stops that motion in one step.
    bool limitDissipation = true;
};

// World space; torques about the centre of mass.
struct BuoyancyState
{
    f32 submergedVolume = 0.0f;
    f32 submergedFraction = 0.0f;
    Math::vec3 centreOfBuoyancy{0.0f};
    f32 waterplaneArea = 0.0f;
    f32 wettedArea = 0.0f;
    // Depth of the deepest hull vertex below the local surface (negative when clear).
    f32 draft = 0.0f;
    u32 wettedTriangles = 0;

    Math::vec3 hydrostaticForce{0.0f};
    Math::vec3 hydrostaticTorque{0.0f};
    Math::vec3 pressureDragForce{0.0f};
    Math::vec3 frictionForce{0.0f};
    Math::vec3 slamForce{0.0f};
    Math::vec3 dynamicTorque{0.0f};
    Math::vec3 force{0.0f};
    Math::vec3 torque{0.0f};

    // Wetted-area mean of the water velocity at the hull.
    Math::vec3 waterVelocity{0.0f};
    // The centre of mass relative to that water.
    f32 speedThroughWater = 0.0f;
    f32 reynolds = 0.0f;
    f32 frictionCoefficient = 0.0f;
    // 1 when the dissipation limit did not act, else the share it let through.
    f32 linearLimit = 1.0f;
    f32 angularLimit = 1.0f;
};

// Hydrostatic and hydrodynamic loads of a closed hull mesh on a rigid body. Each
// evaluation transforms the mesh to world, samples the water height and velocity at every
// vertex and clips each triangle (clipTriangleToWater).
//
// Hydrostatics: p = rho g h is linear over a piece, so its integral is exact: force
// -rho g A hbar n (hbar the mean corner depth) at the centre of pressure
//   x_p = (sum(h) sum(x) + sum(h_i x_i)) / (4 sum(h)),
// from integral(l_i l_j) = A (1 + delta_ij) / 12. Force and moment are exact, so a closed
// mesh in flat water gives Archimedes to rounding. In waves the depth is only piecewise
// linear between vertices, so the hull wants triangles much smaller than a wavelength
// (Froude-Krylov: undisturbed wave pressure, no diffraction or radiation).
//
// Submerged volume, centre of buoyancy and waterplane area follow from the same pieces by
// the divergence theorem. Hydrodynamics: see HydrodynamicCoefficients.
//
// Not modelled: added mass (natural periods are shorter than real by sqrt(1 + added / own
// mass)), radiation damping other than the linear pressure term, wave making and wake,
// spray, the hull's effect on the water.
class BuoyancyBody
{
public:
    BuoyancyBody() = default;
    BuoyancyBody(RigidBody* body, const HullMesh& hull, const WaterSurface* water);

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    RigidBody* body() const
    {
        return mBody;
    }
    // Copies the mesh; resets the slamming history.
    void setHull(const HullMesh& hull);
    const HullMesh& hull() const
    {
        return mHull;
    }
    void setWater(const WaterSurface* water)
    {
        mWater = water;
    }
    const WaterSurface* water() const
    {
        return mWater;
    }

    void setCoefficients(const HydrodynamicCoefficients& coefficients);
    const HydrodynamicCoefficients& coefficients() const
    {
        return mCoefficients;
    }

    // Body-space direction the reference length is measured along (the hull's forward axis),
    // and that length; zero (the default) takes the hull's extent along the axis.
    void setForwardAxis(const Math::vec3& forward);
    const Math::vec3& forwardAxis() const
    {
        return mForward;
    }
    void setReferenceLength(f32 length);
    f32 referenceLength() const;

    // `step` is the duration the loads will act for: it sets the slamming rate and the
    // dissipation limit.
    void applyForces(f32 step);
    // The same without applying or advancing any state (no slamming, no dissipation limit).
    // Updates state().
    void computeForces(Math::vec3& force, Math::vec3& torque);

    const BuoyancyState& state() const
    {
        return mState;
    }
    // Wetted area of one mesh triangle in the last evaluation, and the waterline as pairs of
    // world points.
    f32 wettedArea(u32 triangle) const
    {
        return triangle < mWetted.size() ? mWetted[triangle] : 0.0f;
    }
    const ct::Vector<Math::vec3>& waterline() const
    {
        return mWaterline;
    }

private:
    void evaluate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque);

    RigidBody* mBody = nullptr;
    const WaterSurface* mWater = nullptr;
    HullMesh mHull;
    HydrodynamicCoefficients mCoefficients;
    Math::vec3 mForward{0.0f, 0.0f, 1.0f};
    f32 mReferenceLength = 0.0f;
    f32 mHullLength = 0.0f;
    f32 mHullHalfBeam = 0.0f;
    BuoyancyState mState;

    ct::Vector<WetVertex> mVertices;
    // Per triangle: wetted area now and at the last applied step.
    ct::Vector<f32> mWetted;
    ct::Vector<f32> mPreviousWetted;
    bool mHistoryValid = false;
    ct::Vector<Math::vec3> mWaterline;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_BUOYANCYBODY_H
