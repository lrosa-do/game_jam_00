#ifndef KINETIX_PHYSICS_ACTUATORS_HYDRAULICCYLINDER_H
#define KINETIX_PHYSICS_ACTUATORS_HYDRAULICCYLINDER_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

class RigidBody;

// SI units, except the pump flow (L/min).
struct HydraulicCylinderParameters
{
    f32 boreDiameter = 0.10f;
    f32 rodDiameter = 0.05f;
    // Identical cylinders ganged on one oil line: their areas add, and they
    // are simulated as one row on their mean line.
    u32 count = 1;
    // Pin-to-pin length fully retracted, and the stroke from there to fully
    // extended. A closed length <= 0 takes the pins' distance at
    // construction minus `initialStroke`.
    f32 closedLength = 0.0f;
    f32 stroke = 1.0f;
    f32 initialStroke = 0.0f;
    // Flow the valve can give this cylinder fully open, L/min.
    f32 pumpFlow = 100.0f;
    // Relief valve setting, Pa: caps the push at p * A_cap and the pull at
    // p * A_annulus.
    f32 reliefPressure = 25.0e6f;
    // End-of-stroke cushions: over the last `cushionLength` metres at each
    // end the outflow is throttled, the speed falling linearly to
    // `cushionSpeedFraction` of the commanded one at the stop.
    f32 cushionLength = 0.03f;
    f32 cushionSpeedFraction = 0.15f;
    // Valve command below this (of the -1..1 range) is the spool's overlap:
    // no flow.
    f32 valveDeadband = 0.02f;
    // Time constant of the spool following its command, s (first-order lag;
    // 0 = instant, which hits the bodies with a step in speed).
    f32 valveResponseTime = 0.05f;
    // Oil compressibility: bulk modulus, Pa (mineral oil ~1.5e9; 0 = rigid
    // column), the trapped hose volume per side, m^3, and the damping ratio
    // of the oil spring this makes.
    f32 bulkModulus = 0.0f;
    f32 deadVolume = 5.0e-4f;
    f32 compressibilityDamping = 0.7f;
};

// A hydraulic cylinder acting between a pin on one body (the barrel) and a
// pin on another (the rod eye), as a constraint along the line between them.
// Acts between two bodies, not as a slider: a two-force member across the
// hinge, so the hinges stay hinges and the force has the lever arm the pins
// give it.
//
//  - The valve command u (-1..1, + extends) meters the pump flow Q. Extending
//    fills the cap side: v = u*Q / A_cap; retracting fills the annulus:
//    v = u*Q / A_annulus.
//  - The oil column is the cylinder's position: the stroke integrated from the
//    flow. The row holds the piston there (a rigid servo, or the oil spring
//    k = beta * (A_cap^2 / V_cap + A_ann^2 / V_ann)), so a closed valve holds
//    its load with no drift.
//  - The row's force is bounded to [-p*A_annulus, p*A_cap] by the relief
//    pressure. A load that needs more opens the relief: the piston yields and
//    the oil column follows it. Against an immovable load it stalls at
//    p*A_cap.
//  - Cushions throttle the last centimetres; the stroke ends are hard stops.
class HydraulicCylinder final : public Joint
{
public:
    HydraulicCylinder(RigidBody& barrelBody, const Math::vec3& worldBarrelPin, RigidBody& rodBody,
                      const Math::vec3& worldRodPin, const HydraulicCylinderParameters& parameters);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    // Articulation (Joint::setArticulated): the row and its end stops are
    // solved with the inertia of the whole articulation.
    bool supportsArticulation() const override
    {
        return true;
    }
    u32 articulationMotors(ArticulationMotor* motors) override;
    void setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass) override;
    f32 solveArticulationMotor(u32 id, f32 velocity) override;
    u32 articulationPositionRows(JointRow* rows, f32* errors) override;

    // -1..1, + extends. Values outside are clamped; non-finite ones ignored.
    // Puts the cylinder back on its own valve after setFlow().
    void setValve(f32 command);
    // The flow a shared circuit meters in this step, L/min, + into the cap
    // side. Replaces the own valve, spool lag and deadband until setValve().
    void setFlow(f32 litresPerMinute);
    bool flowControlled() const
    {
        return mFlowControlled;
    }
    // The pressure the supply can reach this step, Pa: the pump's, when it
    // is lower than the cylinder's own relief. <= 0 (or infinite) removes
    // it - a closed spool holds against the port relief alone.
    void setPressureLimit(f32 pascals);
    f32 effectivePressureLimit() const;
    f32 valve() const
    {
        return mValve;
    }
    f32 spool() const
    {
        return mSpool;
    }

    const HydraulicCylinderParameters& parameters() const
    {
        return mParameters;
    }
    f32 capArea() const
    {
        return mCapArea;
    }
    f32 annulusArea() const
    {
        return mAnnulusArea;
    }
    // Ratings: the speeds of the full pump flow (m/s) and the forces at the
    // relief pressure (N).
    f32 maxExtendSpeed() const;
    f32 maxRetractSpeed() const;
    f32 maxPushForce() const;
    f32 maxPullForce() const;

    // Pin-to-pin length (m), stroke from fully retracted (m), its rate (m/s).
    f32 length() const;
    f32 stroke() const;
    f32 strokeVelocity() const;
    Math::vec3 barrelPinWorld() const;
    Math::vec3 rodPinWorld() const;
    f32 oilStroke() const
    {
        return mOilStroke;
    }
    f32 commandedSpeed() const
    {
        return mCommandedSpeed;
    }
    // Force along the cylinder, N: positive pushing the pins apart.
    f32 force() const;
    // The chamber pressures that force means (gauge, Pa), the other side
    // drained to tank.
    f32 capPressure() const;
    f32 rodPressure() const;
    // Flow taken from the valve, L/min (the supply side; returning oil is not
    // reported).
    f32 flow() const;
    bool reliefOpen() const;
    // Force on the end stops, N, positive at the retracted stop.
    f32 endStopForce() const;
    // Whether the piston is at the end of its stroke in this direction (+
    // extending): the pressure then rises to the relief.
    bool atStrokeEnd(f32 direction) const;

    // Sensors (Joint::reactionForce conventions): the force on the rod-eye
    // body at its pin.
    Math::vec3 reactionForce() const override;

private:
    void calculateGeometry();
    f32 rowVelocity() const;
    void applyImpulse(f32 impulse);

    RigidBody* mBarrel;
    RigidBody* mRod;
    Math::vec3 mLocalBarrelPin;
    Math::vec3 mLocalRodPin;
    HydraulicCylinderParameters mParameters;
    f32 mCapArea = 0.0f;
    f32 mAnnulusArea = 0.0f;

    f32 mValve = 0.0f;
    f32 mSpool = 0.0f;
    bool mFlowControlled = false;
    f32 mFlowCommand = 0.0f;
    f32 mPressureLimit = 0.0f;
    f32 mOilStroke = 0.0f;
    f32 mCommandedSpeed = 0.0f;

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::vec3 mAxis{1.0f, 0.0f, 0.0f};
    f32 mLength = 0.0f;
    f32 mInverseEffectiveMass = 0.0f;
    bool mCompressible = false;

    JointDrive mDrive;

    bool mStopActive = false;
    bool mStopAtClosed = true;
    f32 mStopEffectiveMass = 0.0f;
    f32 mTotalStopImpulse = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ACTUATORS_HYDRAULICCYLINDER_H
