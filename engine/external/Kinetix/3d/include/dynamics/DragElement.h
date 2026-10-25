#ifndef KINETIX_PHYSICS_DYNAMICS_DRAGELEMENT_H
#define KINETIX_PHYSICS_DYNAMICS_DRAGELEMENT_H

#include <kinetix/core/Math.h>

namespace kinetix::Physics
{

// Bluff-body drag for everything that is not a wing or propeller:
// drag = 1/2 rho |V| (CdA_x V_x, CdA_y V_y, CdA_z V_z) in body axes.
// dragAreas are Cd*A in m^2 (broadside, from above, head on); applied at `position`.
struct DragElement
{
    Math::vec3 position{0.0f};
    Math::vec3 dragAreas{0.0f};
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_DRAGELEMENT_H
