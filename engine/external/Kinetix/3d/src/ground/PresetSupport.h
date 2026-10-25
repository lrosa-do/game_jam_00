#ifndef KINETIX_PHYSICS_GROUND_PRESETSUPPORT_H
#define KINETIX_PHYSICS_GROUND_PRESETSUPPORT_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/RigidBody.h"
#include "ground/GroundVehicle.h"

namespace kinetix::Physics
{

namespace preset
{

constexpr f32 kGravity = 9.81f;

// Body Y of an axle's wheel centres at full bump that puts the centre of mass `restHeight`
// above flat ground with this axle carrying `axleLoad` (N, both wheels) at rest: the wheel
// sits travel - x_static below full bump and a radius above the ground.
inline f32 axleHeightFor(const AxleParameters& axle, f32 axleLoad, f32 restHeight)
{
    const SuspensionParameters& s = axle.wheel.suspension;
    const f32 length = s.travel - s.staticCompression(0.5f * axleLoad);
    return axle.wheel.radius + length - restHeight;
}

inline void setupBody(RigidBody& body, f32 mass, const Math::vec3& inertiaHalfExtents)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, inertiaHalfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
}

} // namespace preset

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_PRESETSUPPORT_H
