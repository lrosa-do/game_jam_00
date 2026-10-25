#ifndef KINETIX_PHYSICS_GROUND_TRANSMISSION_H
#define KINETIX_PHYSICS_GROUND_TRANSMISSION_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Dry friction plate: capacity maxTorque * engagement; locked within it, slipping at
// exactly the capacity beyond it (a bounded constraint in the drivetrain solve).
struct ClutchParameters
{
    f32 maxTorque = 300.0f; // N m, fully engaged
};

class Clutch
{
public:
    Clutch() = default;
    explicit Clutch(const ClutchParameters& parameters);

    void setParameters(const ClutchParameters& parameters);
    const ClutchParameters& parameters() const
    {
        return mParameters;
    }

    void setEngagement(f32 engagement);
    f32 engagement() const
    {
        return mEngagement;
    }
    f32 capacity() const
    {
        return mParameters.maxTorque * mEngagement;
    }

    // Written by the drivetrain solve: torque engine to gearbox (N m), slip speed engine minus
    // gearbox (rad/s), whether the plate held.
    f32 torque() const
    {
        return mTorque;
    }
    f32 slip() const
    {
        return mSlip;
    }
    bool locked() const
    {
        return mLocked;
    }
    void setSolved(f32 torque, f32 slip, bool locked);

private:
    ClutchParameters mParameters;
    f32 mEngagement = 1.0f;
    f32 mTorque = 0.0f;
    f32 mSlip = 0.0f;
    bool mLocked = true;
};

enum class GearboxMode : u8
{
    Manual,
    Automatic
};

// Ratios are input over output; reverseRatio is given positive. Overall reduction from
// gearbox input to the first differential's input is ratio(gear) * finalDrive.
//
// Efficiency: loss torque at the input of (1 - efficiency) times the torque through it plus
// a constant drag, always opposing the input's rotation.
//
// Automatic: shifts on the speed the input would have in the current gear (a slipping
// launch clutch hides engine speed), up above the upshift speed, down below the downshift
// one, each interpolated between light and full throttle, never within minShiftInterval of
// the last shift. A shift takes shiftTime in neutral; at its end the box engages the gear
// that fits if the target would overspeed.
struct GearboxParameters
{
    static constexpr u32 MaxGears = 18;
    f32 ratios[MaxGears] = {3.6f, 2.1f, 1.4f, 1.03f, 0.8f};
    u32 gearCount = 5;
    f32 reverseRatio = 3.5f;
    f32 finalDrive = 4.1f;
    f32 efficiency = 0.96f;
    f32 dragTorque = 0.5f;    // N m at the input
    f32 inputInertia = 0.02f; // clutch disc and input shaft, kg m^2
    f32 shiftTime = 0.25f;    // s in neutral per shift

    f32 upshiftRpmLight = 2500.0f;
    f32 upshiftRpmFull = 6200.0f;
    f32 downshiftRpmLight = 1200.0f;
    f32 downshiftRpmFull = 3000.0f;
    f32 minShiftInterval = 1.0f; // s

    void setRatios(const f32* forward, u32 count);
};

class Gearbox
{
public:
    // Gear numbers: -1 reverse, 0 neutral, 1..gearCount forward.
    Gearbox() = default;
    explicit Gearbox(const GearboxParameters& parameters);

    void setParameters(const GearboxParameters& parameters);
    const GearboxParameters& parameters() const
    {
        return mParameters;
    }

    void setMode(GearboxMode mode);
    GearboxMode mode() const
    {
        return mMode;
    }

    // In automatic mode any forward gear means "drive". Out-of-range requests are clamped.
    void selectGear(s32 gear);
    void shiftUp();
    void shiftDown();
    // Puts a gear in mesh at once, with no shift time: for a vehicle that is set down at rest.
    void engage(s32 gear);

    // The gear in mesh (0 while shifting), and the one the box is going to.
    s32 gear() const
    {
        return mGear;
    }
    s32 targetGear() const
    {
        return mTarget;
    }
    bool shifting() const
    {
        return mShiftTimer > 0.0f;
    }
    f32 timeSinceShift() const
    {
        return mSinceShift;
    }
    u32 shiftCount() const
    {
        return mShiftCount;
    }

    // Overall ratio of a gear including the final drive: negative for
    // reverse, zero for neutral.
    f32 ratioOf(s32 gear) const;
    f32 ratio() const
    {
        return ratioOf(mGear);
    }

    // Advances a shift in progress and, in automatic mode, decides the next
    // one. outputSpeed is the gearbox output shaft speed to decide on (rad/s,
    // the first differential's input; GroundVehicle passes the speed it
    // would have with no driven tyre slipping, so wheelspin does not look
    // like road speed), throttle the driver's 0..1. canShiftUp is false
    // while a driven wheel slips or is off the ground, or the engine is
    // slowing: an upshift then waits, a downshift does not.
    void update(f32 step, f32 outputSpeed, f32 throttle, bool canShiftUp = true);

    f32 inputAngularVelocity() const
    {
        return mInputOmega;
    }
    void setInputAngularVelocity(f32 omega);

    f32 torque() const
    {
        return mTorque;
    }
    void setSolvedTorque(f32 torque);

private:
    void beginShift(s32 target);
    s32 automaticGearFor(f32 outputSpeed, f32 throttle) const;

    GearboxParameters mParameters;
    GearboxMode mMode = GearboxMode::Manual;
    s32 mGear = 0;
    s32 mTarget = 0;
    f32 mShiftTimer = 0.0f;
    f32 mSinceShift = 1.0e6f;
    u32 mShiftCount = 0;
    f32 mInputOmega = 0.0f;
    f32 mOutputOmega = 0.0f;
    f32 mTorque = 0.0f;
};

enum class DifferentialType : u8
{
    // Equal torque to both outputs (or split / 1 - split), free speed difference.
    Open,
    // A spool: both outputs turn together.
    Locked,
    // Friction plates holding the outputs together until the torque carried reaches
    // preloadTorque + torqueSensing * |carrier torque|, then slipping at exactly that (output
    // torques then differ by twice it).
    ClutchLimitedSlip,
    // Carries viscousCoefficient times the speed difference, from the faster to the slower.
    ViscousLimitedSlip
};

// Kinematics: omega_in / ratio = split * omega_A + (1 - split) * omega_B; output A gets
// split times the carrier torque, B the rest (0.5 for an ordinary axle differential). A
// lockable one becomes a spool while locked, whatever its type.
struct DifferentialParameters
{
    DifferentialType type = DifferentialType::Open;
    f32 ratio = 1.0f;
    f32 split = 0.5f;
    f32 preloadTorque = 0.0f;
    f32 torqueSensing = 0.0f;
    f32 viscousCoefficient = 0.0f; // N m s / rad
    bool lockable = false;
    f32 inputInertia = 0.05f; // input shaft and carrier, kg m^2
};

struct DriveOutput
{
    enum class Kind : u8
    {
        None,
        Wheel,
        Differential
    };
    Kind kind = Kind::None;
    u32 index = 0;

    static DriveOutput wheel(u32 index)
    {
        DriveOutput out;
        out.kind = Kind::Wheel;
        out.index = index;
        return out;
    }
    static DriveOutput differential(u32 index)
    {
        DriveOutput out;
        out.kind = Kind::Differential;
        out.index = index;
        return out;
    }
};

class Differential
{
public:
    Differential() = default;
    Differential(const DifferentialParameters& parameters, DriveOutput a, DriveOutput b);

    void setParameters(const DifferentialParameters& parameters);
    const DifferentialParameters& parameters() const
    {
        return mParameters;
    }
    DriveOutput outputA() const
    {
        return mA;
    }
    DriveOutput outputB() const
    {
        return mB;
    }

    // Ignored unless the differential is lockable.
    void setLocked(bool locked);
    bool locked() const
    {
        return mLocked && mParameters.lockable;
    }
    // Acts as a spool this step: Locked type, or a lockable one locked.
    bool rigid() const
    {
        return mParameters.type == DifferentialType::Locked || locked();
    }

    f32 inputAngularVelocity() const
    {
        return mInputOmega;
    }
    void setInputAngularVelocity(f32 omega);

    // Written by the drivetrain solve (N m): torque into the input, and the
    // torque the locking element moved from the faster output to the slower.
    f32 inputTorque() const
    {
        return mInputTorque;
    }
    f32 lockingTorque() const
    {
        return mLockingTorque;
    }
    void setSolved(f32 inputTorque, f32 lockingTorque);

private:
    DifferentialParameters mParameters;
    DriveOutput mA;
    DriveOutput mB;
    bool mLocked = false;
    f32 mInputOmega = 0.0f;
    f32 mInputTorque = 0.0f;
    f32 mLockingTorque = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_TRANSMISSION_H
