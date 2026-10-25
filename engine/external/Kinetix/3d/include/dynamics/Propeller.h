#ifndef KINETIX_PHYSICS_DYNAMICS_PROPELLER_H
#define KINETIX_PHYSICS_DYNAMICS_PROPELLER_H

#include "dynamics/Airfoil.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class Atmosphere;
class RigidBody;

// Steady loads of a propeller at one operating point.
struct PropellerLoads
{
    f32 thrust = 0.0f; // N along the propeller axis, negative when braking
    f32 torque = 0.0f; // N m the air exerts against the rotation (negative: windmilling)
    f32 power = 0.0f;  // W absorbed from the shaft, torque * omega
};

// A fixed-pitch propeller solved with blade element momentum theory (BEMT).
//
// In each annulus the blade element sees axial inflow V0 + v and tangential
// speed omega r - w (v, w: induced axial and swirl velocities). Its lift and
// drag give the annulus' thrust and torque, which must equal momentum theory's
//   dT = 4 pi r rho F (V0 + v) v dr         dQ = 4 pi r^2 rho F (V0 + v) w dr
// with F the Prandtl tip and hub loss factor. v is found by false position in
// a bracket that always exists (v = -V0 / 2 up to a speed no induced velocity
// reaches), so it converges at zero airspeed where the induction-factor form
// (a = v / V0) divides by zero. w is then updated from the torque and v solved
// again, three times (within ~1% of converged).
//
// Only the axial inflow is used: no in-plane component (P-factor, normal
// force) and no slipstream over the surfaces behind. Flow from behind the disc
// (V0 < 0) is treated as static; a windmilling disc past the momentum-theory
// limit is held at v = -V0 / 2.
//
// Rotor speed is either set directly (rpm mode) or integrated from shaft torque
// against the aerodynamic torque through the rotor's inertia (torque mode). The
// airframe feels thrust, the shaft torque reaction and the gyroscopic torque.
class Propeller
{
public:
    struct Description
    {
        f32 radius = 0.95f;
        f32 hubRadius = 0.15f;
        u32 bladeCount = 2;
        f32 rootChord = 0.15f; // at the hub
        f32 tipChord = 0.08f;
        // Geometric pitch: advance per revolution at which every station meets
        // the air at zero geometric angle of attack; blade angle is
        // atan(pitch / (2 pi r)) + twistOffset.
        f32 pitch = 1.5f;
        f32 twistOffset = 0.0f;
        // Thrust direction and hub position, body space.
        Math::vec3 position{0.0f};
        Math::vec3 axis{0.0f, 0.0f, 1.0f};
        // +1 spins right-handed about `axis` (clockwise seen from behind), -1 the other way.
        f32 rotation = 1.0f;
        // Rotor plus whatever turns with it, about the axis, kg m^2.
        f32 inertia = 2.5f;
        // Annuli, 2..64.
        u32 elementCount = 12;
    };

    enum class Mode : u8
    {
        Rpm,
        Torque
    };

    Propeller();
    Propeller(const Description& description, const Airfoil& airfoil);

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

    // Rpm mode: the rotor turns at exactly this speed.
    void setRpm(f32 rpm);
    // Torque mode: the shaft torque driving the rotor, integrated each step.
    void setShaftTorque(f32 torque);
    Mode mode() const
    {
        return mMode;
    }
    f32 rpm() const;
    f32 angularSpeed() const
    {
        return mOmega;
    }
    // Sets the speed without leaving torque mode (a starting condition).
    void setAngularSpeed(f32 omega);
    f32 shaftTorque() const
    {
        return mShaftTorque;
    }

    // Pure BEMT: loads at angular speed omega (rad/s, >= 0), air arriving along
    // the axis at axialSpeed (m/s, positive from ahead).
    PropellerLoads evaluate(f32 omega, f32 axialSpeed, f32 density) const;

    // Evaluates the loads at the body's current state, advances the rotor speed
    // (torque mode) and adds thrust and reaction torque to the body; `step` is
    // how long the forces act. The world force and torque added go to the
    // optional outputs.
    const PropellerLoads& apply(RigidBody& body, const Atmosphere& atmosphere, f32 step,
                                Math::vec3* appliedForce = nullptr,
                                Math::vec3* appliedTorque = nullptr);
    // Same evaluation without touching the body or rotor speed; world space,
    // torque about the centre of mass.
    PropellerLoads computeForces(const RigidBody& body, const Atmosphere& atmosphere,
                                 Math::vec3& force, Math::vec3& torque) const;

    f32 axialInflow(const RigidBody& body, const Atmosphere& atmosphere) const;

    const PropellerLoads& loads() const
    {
        return mLoads;
    }
    // J = V / (n D).
    f32 advanceRatio(f32 omega, f32 axialSpeed) const;

private:
    struct Station
    {
        f32 radius;
        f32 width;
        f32 chord;
        f32 bladeAngle;
    };
    static constexpr u32 MaxStations = 64;

    void rebuild();

    Description mDescription;
    Airfoil mAirfoil;
    Station mStations[MaxStations];
    u32 mStationCount = 0;
    Mode mMode = Mode::Rpm;
    f32 mOmega = 0.0f;
    f32 mShaftTorque = 0.0f;
    PropellerLoads mLoads;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_PROPELLER_H
