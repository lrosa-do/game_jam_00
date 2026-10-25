#ifndef KINETIX_PHYSICS_GROUND_ENGINE_H
#define KINETIX_PHYSICS_GROUND_ENGINE_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Net (brake) torque curve; combustion = throttle * (curve + friction), friction always
// opposes the shaft. Soft-cut rev limiter fades fuel over limiterBand below limiterRpm.
// The crank speed is not integrated here: GroundVehicle solves the drivetrain and writes
// it back with setAngularVelocity().
struct EngineParameters
{
    static constexpr u32 MaxCurvePoints = 24;
    // Full-load (net) torque curve, rpm ascending. Below the first point the first
    // torque is held; past the last the torque is zero (a governed diesel's
    // high idle, or a petrol engine's valve float).
    f32 curveRpm[MaxCurvePoints] = {};
    f32 curveTorque[MaxCurvePoints] = {};
    u32 curvePoints = 0;

    f32 idleRpm = 800.0f;
    f32 stallRpm = 400.0f;
    f32 limiterRpm = 6500.0f;
    f32 limiterBand = 100.0f;
    // Crankshaft, flywheel and clutch pressure plate, kg m^2.
    f32 inertia = 0.15f;
    // Friction and pumping torque, N m: static + perRpm * rpm + perRpm2 * rpm^2.
    f32 frictionTorque = 10.0f;
    f32 frictionPerRpm = 0.0035f;
    f32 frictionPerRpm2 = 0.0f;
    // Idle control: extra throttle per unit of fractional speed deficit
    // below idle, up to maxIdleThrottle.
    f32 idleGain = 4.0f;
    f32 maxIdleThrottle = 0.35f;
    f32 starterTorque = 40.0f;
    f32 starterCatchRpm = 300.0f;

    void setTorqueCurve(const f32* rpm, const f32* torque, u32 count);

    static EngineParameters gasoline(f32 maxPower, f32 powerRpm, f32 maxTorque, f32 torqueRpm,
                                     f32 idleRpm, f32 limiterRpm);
    static EngineParameters diesel(f32 maxPower, f32 powerRpm, f32 maxTorque, f32 torqueRpm,
                                   f32 plateauEndRpm, f32 idleRpm, f32 highIdleRpm);
};

class Engine
{
public:
    Engine();
    explicit Engine(const EngineParameters& parameters);

    void setParameters(const EngineParameters& parameters);
    const EngineParameters& parameters() const
    {
        return mParameters;
    }

    f32 fullLoadTorque(f32 rpm) const;
    f32 frictionTorque(f32 rpm) const;
    f32 peakPower(f32* atRpm = nullptr) const;

    // One step of the engine's control: stall detection, starter, idle
    // control and rev limiter, for the driver's throttle (0..1). Returns the
    // torque combustion and the starter put on the crank this step (>= 0);
    // friction is the drivetrain's to apply, as a resisting torque bounded by
    // frictionTorque() that never drives the crank backwards.
    f32 update(f32 throttle, bool starter);

    void start();
    void stop();
    bool running() const
    {
        return mRunning;
    }

    f32 angularVelocity() const
    {
        return mOmega;
    }
    void setAngularVelocity(f32 omega);
    f32 rpm() const;

    f32 effectiveThrottle() const
    {
        return mEffectiveThrottle;
    }
    bool limiterActive() const
    {
        return mLimiterCut;
    }
    f32 combustionTorque() const
    {
        return mCombustionTorque;
    }

private:
    EngineParameters mParameters;
    f32 mOmega = 0.0f;
    bool mRunning = true;
    bool mLimiterCut = false;
    f32 mEffectiveThrottle = 0.0f;
    f32 mCombustionTorque = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_ENGINE_H
