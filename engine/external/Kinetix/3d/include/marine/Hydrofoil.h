#ifndef KINETIX_PHYSICS_MARINE_HYDROFOIL_H
#define KINETIX_PHYSICS_MARINE_HYDROFOIL_H

#include "dynamics/Airfoil.h"
#include "dynamics/Wing.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// A Wing (strip theory, lifting-line downwash) evaluated in water: the water's density, its
// velocity at each strip (orbital motion and current) and per strip the share of span under
// the surface. A strip half out carries half its load; one clear carries nothing and does
// not count towards the foil's lift coefficient. Immersion is measured along the span from
// the water height at the two ends, blended over a tenth of a chord either side of the
// surface.
//
// setDeflection() turns the whole foil about its quarter-chord span axis (spade rudder,
// pivoting centreboard, outboard leg steering), adding to the Wing's described incidence;
// the wing's own control surfaces still work as flaps on top.
//
// Not modelled: loss of lift near the free surface (within about a chord), ventilation and
// cavitation.
class Hydrofoil
{
public:
    Hydrofoil();
    Hydrofoil(const Wing::Description& description, const Airfoil& airfoil);

    Wing& wing()
    {
        return mWing;
    }
    const Wing& wing() const
    {
        return mWing;
    }
    // Replaces the geometry; the deflection is kept and added to it.
    void setDescription(const Wing::Description& description);
    const Wing::Description& description() const
    {
        return mBase;
    }

    // Radians, clamped to +-maxDeflection(). Positive turns the leading edge
    // towards the description's `up`, i.e. more lift along it.
    void setDeflection(f32 angle);
    f32 deflection() const
    {
        return mDeflection;
    }
    void setMaxDeflection(f32 angle);
    f32 maxDeflection() const
    {
        return mMaxDeflection;
    }

    const WingState& computeForces(const RigidBody& body, const WaterSurface& water);
    const WingState& apply(RigidBody& body, const WaterSurface& water);

    f32 immersion() const
    {
        return mImmersion;
    }
    f32 elementImmersion(u32 element) const
    {
        return element < kMaxElements ? mScales[element] : 0.0f;
    }

private:
    static constexpr u32 kMaxElements = 128;

    void rebuild();
    void measureImmersion(const RigidBody& body, const WaterSurface& water);

    Wing mWing;
    Wing::Description mBase;
    f32 mDeflection = 0.0f;
    f32 mMaxDeflection = 0.6f;
    f32 mImmersion = 0.0f;
    f32 mScales[kMaxElements] = {};
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_HYDROFOIL_H
