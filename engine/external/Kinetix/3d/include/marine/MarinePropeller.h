#ifndef KINETIX_PHYSICS_MARINE_MARINEPROPELLER_H
#define KINETIX_PHYSICS_MARINE_MARINEPROPELLER_H

#include "dynamics/Propeller.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// Screw propeller in water behind a reversing gearbox. The blades are a Propeller (BEMT)
// evaluated with the water's density.
//
//  - Gearbox: the engine turns gearRatio times faster than the propeller, through
//    forward / neutral / reverse. In gear engine and propeller are one rigid shaft (inertia
//    referred to the engine), so an engine-driven propeller has one state, the ENGINE speed,
//    integrated from engine torque against propeller torque reflected through the gearbox
//    (gearEfficiency). In neutral the engine spins free and the propeller is held still (a
//    locked propeller drags a little).
//  - Astern: BEMT with forward-running section data cannot describe a propeller turning
//    backwards, so loads are those of the propeller running ahead at the same speed into a
//    flow of opposite sign, thrust reversed and scaled by reverseEfficiency (0.5-0.7 for
//    fixed pitch), torque reversed with the rotation.
//  - Hull interaction: the water reaches the propeller at (1 - w) of the boat's speed (wake
//    fraction w); suction raises hull resistance as a thrust deduction t: the boat feels
//    (1 - t) T.
//  - Immersion: thrust and torque scale with the share of the disc under the local surface.
//    A propeller clear of the water produces nothing and the engine races to its limiter.
//    (Real ventilation starts before the disc breaks the surface.)
//  - Steering: the thrust line turns about steeringAxis through the hub (outboard or
//    sterndrive leg) by up to maxSteeringAngle.
//
// The hull feels the thrust at the hub along the (steered) axis and the reaction to the
// propeller's hydrodynamic torque. Gyroscopic torque and cavitation are neglected.
class MarinePropeller
{
public:
    enum class Gear : s8
    {
        Reverse = -1,
        Neutral = 0,
        Forward = 1
    };

    struct Description
    {
        Propeller::Description blades;
        f32 gearRatio = 2.0f;        // engine turns per propeller turn
        f32 gearEfficiency = 0.95f;
        f32 reverseEfficiency = 0.6f;
        f32 wakeFraction = 0.08f;
        f32 thrustDeduction = 0.06f;
        // Engine and flywheel about the crankshaft, kg m^2 (the propeller's is blades.inertia).
        f32 engineInertia = 0.12f;
        Math::vec3 steeringAxis{0.0f, 1.0f, 0.0f};
        f32 maxSteeringAngle = 0.0f; // radians
    };

    MarinePropeller();
    MarinePropeller(const Description& description, const Airfoil& airfoil);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }
    const Propeller& blades() const
    {
        return mBlades;
    }

    void setGear(Gear gear);
    Gear gear() const
    {
        return mGear;
    }
    // Torque the engine delivers to the gearbox, N m; set every step from the engine at
    // engineAngularSpeed().
    void setEngineTorque(f32 torque)
    {
        mEngineTorque = isfinite(torque) ? torque : 0.0f;
        mHeldRpm = false;
    }
    f32 engineTorque() const
    {
        return mEngineTorque;
    }
    // Holds the propeller at a fixed speed instead (signed: negative astern), whatever it costs.
    void setPropellerRpm(f32 rpm);
    bool holdsRpm() const
    {
        return mHeldRpm;
    }
    // A starting condition: sets the engine speed (and, in gear, the propeller's) without
    // changing the mode.
    void setEngineAngularSpeed(f32 omega);

    f32 engineAngularSpeed() const
    {
        return mEngineOmega;
    }
    f32 engineRpm() const;
    // Signed: positive running ahead.
    f32 angularSpeed() const;
    f32 rpm() const;

    void setSteering(f32 angle);
    f32 steering() const
    {
        return mSteering;
    }

    // Pure hydrodynamics: loads at signed propeller speed omega with the
    // water arriving along the axis at advanceSpeed (positive from ahead,
    // after the wake fraction), scaled by `immersion`. Thrust positive
    // ahead, torque with the sign of the rotation it resists.
    PropellerLoads evaluate(f32 omega, f32 advanceSpeed, f32 density, f32 immersion = 1.0f) const;

    Math::vec3 thrustAxis(const RigidBody& body) const;
    Math::vec3 hubPosition(const RigidBody& body) const;
    f32 immersion(const RigidBody& body, const WaterSurface& water) const;
    // Speed of the water into the disc along the axis, after the wake fraction.
    f32 advanceSpeed(const RigidBody& body, const WaterSurface& water) const;

    // Evaluates, adds thrust and reaction torque to the body and advances the shaft by `step`.
    // The world force and torque added (about the centre of mass) go to the optional outputs.
    const PropellerLoads& apply(RigidBody& body, const WaterSurface& water, f32 step,
                                Math::vec3* appliedForce = nullptr,
                                Math::vec3* appliedTorque = nullptr);
    // The same without applying anything or changing the shaft speed.
    PropellerLoads computeForces(const RigidBody& body, const WaterSurface& water, Math::vec3& force,
                                 Math::vec3& torque) const;

    const PropellerLoads& loads() const
    {
        return mLoads;
    }
    f32 lastImmersion() const
    {
        return mImmersion;
    }

private:
    PropellerLoads loadsAt(const RigidBody& body, const WaterSurface& water, f32 share, Math::vec3& force,
                           Math::vec3& torque) const;
    f32 gearSign() const
    {
        return static_cast<f32>(static_cast<s8>(mGear));
    }

    Description mDescription;
    Propeller mBlades;
    Gear mGear = Gear::Forward;
    f32 mEngineOmega = 0.0f;
    f32 mEngineTorque = 0.0f;
    f32 mSteering = 0.0f;
    bool mHeldRpm = false;
    f32 mHeldOmega = 0.0f; // signed propeller speed in held mode
    PropellerLoads mLoads;
    f32 mImmersion = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_MARINEPROPELLER_H
