#ifndef KINETIX_PHYSICS_MARINE_WATEREFFECTS_H
#define KINETIX_PHYSICS_MARINE_WATEREFFECTS_H

#include <kinetix/core/Math.h>

namespace kinetix::Physics
{

// What a hull and a propeller do to the water, for whoever draws it: each
// call gives a point on or near the surface and the velocity that threw the
// water there.
//
//   splash(p, v)  water thrown up where a point of the hull runs just under
//                 the surface, and in the propeller's wash
//   ripple(p, v)  the ring that spreads on the surface at the same places
//   spray(p, v)   what a submerged face pushing hard through the water
//                 throws off at its edge on the surface
class WaterEffects
{
public:
    virtual ~WaterEffects() = default;
    virtual void splash(const Math::vec3& position, const Math::vec3& velocity) = 0;
    virtual void ripple(const Math::vec3& position, const Math::vec3& velocity) = 0;
    virtual void spray(const Math::vec3& position, const Math::vec3& velocity) = 0;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_WATEREFFECTS_H
