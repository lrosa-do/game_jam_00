#ifndef KINETIX_PHYSICS_DYNAMICS_PISTONENGINE_H
#define KINETIX_PHYSICS_DYNAMICS_PISTONENGINE_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// A normally aspirated piston engine driving a propeller directly: throttle and
// shaft speed in, shaft torque out.
//
// Full-throttle power: P / P_max = x + x^2 - x^3, x = rpm / ratedRpm (peaks at
// the rated speed, finite torque P_max / omega_rated at standstill). Throttle
// scales torque between idle (idleThrottle) and full; friction and pumping
// losses grow linearly with speed; past maxRpm torque is cut over a short band
// (governor). Altitude: Gagg-Ferrar, P / P0 = sigma - (1 - sigma) / 7.55, sigma
// the density ratio. No starter, no stall.
class PistonEngine
{
public:
    struct Description
    {
        f32 maxPower = 120000.0f; // W at ratedRpm, sea level
        f32 ratedRpm = 2700.0f;
        f32 maxRpm = 2900.0f;
        f32 idleThrottle = 0.08f;
        // Friction torque as a share of rated torque: at zero speed and the
        // extra per rated speed.
        f32 frictionStatic = 0.03f;
        f32 frictionPerRpm = 0.07f;
    };

    PistonEngine() = default;
    explicit PistonEngine(const Description& description);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }

    void setThrottle(f32 throttle);
    f32 throttle() const
    {
        return mThrottle;
    }

    // Net shaft torque, N m, at shaft speed omega (rad/s) in air of the given
    // density. Negative when friction outweighs combustion (windmilling idle).
    f32 shaftTorque(f32 omega, f32 density) const;
    // Torque before losses at full throttle and sea level, N m.
    f32 fullThrottleTorque(f32 omega) const;
    // Rated torque, P_max / omega_rated.
    f32 ratedTorque() const;

private:
    Description mDescription;
    f32 mThrottle = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_PISTONENGINE_H
