#ifndef KINETIX_PHYSICS_DYNAMICS_WING_H
#define KINETIX_PHYSICS_DYNAMICS_WING_H

#include "dynamics/Airfoil.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class Atmosphere;
class RigidBody;

// A movable trailing-edge surface on part of a wing (aileron, elevator, rudder,
// flap). The wing only reads the command on each panel; the Aircraft sets it
// from the pilot inputs (see `mix`).
struct ControlSurface
{
    // Share of the panel's span it covers, root (0) to tip (1).
    f32 spanStart = 0.0f;
    f32 spanEnd = 1.0f;
    f32 chordFraction = 0.25f;
    // Deflection at command +1 (trailing edge towards -up, i.e. more lift) and
    // its magnitude at command -1; radians, both positive.
    f32 maxDeflection = 0.35f;
    f32 minDeflection = 0.35f;

    // Command = aileron * mix.aileron + elevator * mix.elevator + rudder *
    // mix.rudder + flaps * mix.flaps, clamped to -1..1. On the mirrored panel the
    // aileron and rudder terms change sign, so one surface can be an aileron,
    // elevator, flap or any mix (elevons, flaperons, ruddervators).
    struct Mix
    {
        f32 aileron = 0.0f;
        f32 elevator = 0.0f;
        f32 rudder = 0.0f;
        f32 flaps = 0.0f;
    } mix;

    // Current command, -1..1, on the primary panel and on the mirrored one.
    f32 command = 0.0f;
    f32 mirroredCommand = 0.0f;

    f32 deflectionFor(f32 normalizedCommand) const
    {
        const f32 c = Clamp(normalizedCommand, -1.0f, 1.0f);
        return c >= 0.0f ? c * maxDeflection : c * minDeflection;
    }
};

// What a wing did on its last evaluation. Forces are world space, torque is
// about the body's centre of mass.
struct WingState
{
    Math::vec3 force{0.0f};
    Math::vec3 torque{0.0f};
    f32 lift = 0.0f;
    f32 drag = 0.0f;
    // Whole-wing lift coefficient (dynamic-pressure weighted mean of the
    // sections') and the induced angle it implies.
    f32 liftCoefficient = 0.0f;
    f32 inducedAngle = 0.0f;
    // Area-weighted share of the wing whose sections are past stall.
    f32 stalledFraction = 0.0f;
    f32 dynamicPressure = 0.0f;
};

// A lifting surface: a straight, possibly tapered and twisted panel between a
// root and a tip, cut into spanwise strips that each fly as an independent
// airfoil section (strip theory), plus an optional copy reflected through the
// aircraft's plane of symmetry.
//
// Per strip, every evaluation:
//   - the air velocity at the quarter-chord point is wind - (v + w x r);
//   - the spanwise component is removed (cosine rule of swept-wing strip
//     theory); the rest sets q = rho V^2 / 2 and the angle of attack against the chord;
//   - Cl, Cd, Cm come from the airfoil at that angle less the induced angle,
//     with the strip's control deflection;
//   - lift acts across the flow in the chordwise plane, drag along it, plus the
//     moment about the quarter chord.
//
// Finite span: all strips share one induced angle alpha_i = CL / (pi e AR), CL
// the whole wing's lift coefficient (uniform downwash of Prandtl's lifting line
// for elliptic loading). It is solved each evaluation by Newton iteration
// (exact in one step in attached flow) and tilts every strip's lift back by
// alpha_i, so the induced drag CL^2 / (pi e AR) comes out of the same force.
// The linear lift slope is then a0 / (1 + a0 / (pi e AR)). Roll damping and
// spanwise load are those of strip theory, overestimated somewhat at low aspect ratio.
//
// A vertical fin is a Wing whose `up` points sideways. On a fuselage it acts
// like half of a wing reflected in the body (end plate), so its effective
// aspect ratio is about 2 b^2 / S - set aspectRatio.
class Wing
{
public:
    struct Description
    {
        // Quarter-chord points at the two ends, body space.
        Math::vec3 root{0.0f};
        Math::vec3 tip{1.0f, 0.0f, 0.0f};
        f32 rootChord = 1.0f;
        f32 tipChord = 1.0f;
        // Radians, positive leading edge towards `up`; linear root to tip
        // (washout: tip incidence below the root's).
        f32 rootIncidence = 0.0f;
        f32 tipIncidence = 0.0f;
        // The body direction the leading edge faces, and the side positive lift
        // points to. Both are made perpendicular to the span (and each other), so
        // dihedral is just a tip higher than the root.
        Math::vec3 forward{0.0f, 0.0f, 1.0f};
        Math::vec3 up{0.0f, 1.0f, 0.0f};
        // Adds the panel reflected through the plane through the body origin
        // with this normal (body X by default: left and right wings).
        bool mirrored = false;
        Math::vec3 mirrorNormal{1.0f, 0.0f, 0.0f};
        // Strips per panel, 1..64.
        u32 elementCount = 8;
        f32 oswaldEfficiency = 0.8f;
        // Effective aspect ratio for the downwash. Zero takes b^2 / S of the
        // wing as built (both panels when mirrored).
        f32 aspectRatio = 0.0f;
    };

    struct Element
    {
        Math::vec3 position{0.0f}; // quarter chord, body space
        Math::vec3 chordAxis{0.0f, 0.0f, 1.0f}; // towards the leading edge
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        // chordAxis x normal: the axis positive (nose-up) moment turns about.
        Math::vec3 pitchAxis{-1.0f, 0.0f, 0.0f};
        f32 chord = 1.0f;
        f32 area = 1.0f;
        f32 spanFraction = 0.5f;
        bool mirroredPanel = false;
        // Control surface over this strip (-1 for none) and the share of the
        // strip's span it covers.
        s32 control = -1;
        f32 controlCoverage = 0.0f;

        f32 angleOfAttack = 0.0f;
        f32 dynamicPressure = 0.0f;
        f32 lift = 0.0f;
        f32 drag = 0.0f;
        f32 separation = 0.0f;
    };

    Wing();
    Wing(const Description& description, const Airfoil& airfoil);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }
    void setAirfoil(const Airfoil& airfoil)
    {
        mAirfoil = airfoil;
    }
    const Airfoil& airfoil() const
    {
        return mAirfoil;
    }

    // Returns the index. Surfaces on one wing should not overlap: a strip takes
    // the one covering most of it.
    u32 addControlSurface(const ControlSurface& surface);
    u32 controlSurfaceCount() const
    {
        return static_cast<u32>(mControls.size());
    }
    ControlSurface& controlSurface(u32 index)
    {
        return mControls[index];
    }
    const ControlSurface& controlSurface(u32 index) const
    {
        return mControls[index];
    }
    // Sets both panels' commands of one surface. An index past
    // controlSurfaceCount() is reported and ignored.
    void setControlCommand(u32 index, f32 command, f32 mirroredCommand);

    // Computes this wing's force and torque on `body` without applying them. Not
    // const: the induced angle is solved from the previous one and the
    // per-element telemetry is updated.
    const WingState& computeForces(const RigidBody& body, const Atmosphere& atmosphere);
    const WingState& apply(RigidBody& body, const Atmosphere& atmosphere);

    // The velocity of the surrounding fluid at a world point: the wind for an
    // aircraft, current plus wave motion for a hydrofoil.
    using FlowField = Math::vec3 (*)(const Math::vec3& worldPoint, const void* userData);
    // The same evaluation in any fluid: `density` throughout, moving as `flow`
    // describes (null for still). `elementScale`, if given, holds one factor per
    // element multiplying that strip's dynamic pressure (the immersed share of a
    // partly immersed surface); a strip at zero carries nothing and does not
    // count towards the wing's lift coefficient.
    const WingState& computeForces(const RigidBody& body, f32 density, FlowField flow,
                                   const void* userData, const f32* elementScale = nullptr);
    const WingState& apply(RigidBody& body, f32 density, FlowField flow, const void* userData,
                           const f32* elementScale = nullptr);

    // Velocity, body space, added to the air every strip sees: the downwash of a
    // surface ahead (Aircraft sets it each evaluation from its downwash links).
    void setInducedFlow(const Math::vec3& bodyVelocity)
    {
        mInducedFlow = bodyVelocity;
    }
    const Math::vec3& inducedFlow() const
    {
        return mInducedFlow;
    }

    const WingState& state() const
    {
        return mState;
    }
    u32 elementCount() const
    {
        return static_cast<u32>(mElements.size());
    }
    const Element& element(u32 index) const
    {
        return mElements[index];
    }

    // Planform area and span as built, both panels when mirrored.
    f32 area() const
    {
        return mArea;
    }
    f32 span() const
    {
        return mSpan;
    }
    f32 aspectRatio() const
    {
        return mAspectRatio;
    }
    // Per radian: the lift slope the whole wing should show in attached flow.
    f32 finiteLiftSlope() const;

private:
    void rebuild();
    void assignControls();

    Description mDescription;
    Airfoil mAirfoil;
    ct::Vector<ControlSurface> mControls;
    ct::Vector<Element> mElements;
    WingState mState;
    f32 mArea = 0.0f;
    f32 mSpan = 0.0f;
    f32 mAspectRatio = 1.0f;
    f32 mInducedAngle = 0.0f;
    Math::vec3 mInducedFlow{0.0f};
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_WING_H
