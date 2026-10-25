#ifndef KINETIX_PHYSICS_ROTORCRAFT_HELICOPTERAUTOPILOT_H
#define KINETIX_PHYSICS_ROTORCRAFT_HELICOPTERAUTOPILOT_H

#include "rotorcraft/Helicopter.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Loop gains. Units: stick (-1..1) per radian or per rad/s, m/s^2 per m/s,
// m/s per m, collective per m/s.
struct HelicopterAutopilotGains
{
    f32 position = 0.35f;        // velocity command per metre of position error
    f32 maxSpeed = 12.0f;        // cap on that command, m/s
    f32 velocity = 0.6f;         // acceleration command per m/s
    f32 velocityIntegral = 0.12f;
    f32 integrationWindow = 2.0f; // m/s: the velocity integrator only runs inside it
    f32 maxAcceleration = 3.0f;  // m/s^2
    f32 attitude = 2.0f;         // cyclic per radian
    f32 attitudeIntegral = 0.6f;
    f32 pitchRate = 0.5f;
    f32 rollRate = 0.3f;
    f32 maxTilt = 0.35f;         // attitude command limit, radians
    f32 heading = 2.0f;          // yaw rate command per radian of heading error
    f32 maxYawRate = 0.5f;       // rad/s
    f32 yawRate = 0.8f;          // pedal per rad/s of yaw rate error
    f32 headingIntegral = 0.4f;  // pedal per radian second
    f32 height = 0.6f;           // vertical speed command per metre
    f32 maxClimb = 3.0f;         // m/s
    f32 verticalSpeed = 0.12f;   // collective per m/s
    f32 verticalIntegral = 0.08f;
    // Rotor speed protection: below this speed (percent) the collective comes down by
    // `rotorSpeedRelief` per percent short of it.
    f32 rotorSpeedFloor = 97.0f;
    f32 rotorSpeedRelief = 0.04f;
};

// Cascaded PI autopilot to hold a helicopter still: position to ground velocity (capped by the
// speed it can still brake from over the distance left), velocity to acceleration (the integrator
// finds the trim attitude, whatever wind or c.g.), acceleration to pitch and bank (a = -g tan(pitch)
// forward, g tan(bank) to the right), attitude to cyclic with rate damping and a trim integrator;
// height (or vertical speed) to collective, backed off when the rotor droops; heading to a capped
// yaw rate, and that to pedals. Reads only the helicopter's telemetry and body; writes
// HelicopterControls.
//
// For tests and the testbed: no gain scheduling, no envelope protection beyond the attitude and
// speed caps.
class HelicopterAutopilot
{
public:
    enum class Horizontal : u8
    {
        Position, // hover over targetPosition
        Velocity  // fly targetVelocity (horizontal, world) over the ground
    };
    enum class Vertical : u8
    {
        Altitude,     // hold targetPosition.y (world)
        VerticalSpeed // climb or descend at targetVerticalSpeed
    };

    HelicopterAutopilot() = default;

    void setGains(const HelicopterAutopilotGains& gains)
    {
        mGains = gains;
    }
    const HelicopterAutopilotGains& gains() const
    {
        return mGains;
    }

    void holdPosition(const Math::vec3& worldPosition, f32 heading);
    void holdVelocity(const Math::vec3& groundVelocity, f32 heading);
    void holdAltitude(f32 worldHeight);
    void holdVerticalSpeed(f32 verticalSpeed);

    Horizontal horizontalMode() const
    {
        return mHorizontal;
    }
    Vertical verticalMode() const
    {
        return mVertical;
    }

    // Starts the integrators from the controls the helicopter is flying with, so engaging is bumpless.
    void engage(const Helicopter& helicopter);
    // Starts them from a trim instead: the attitude and controls the flight about to begin needs
    // (a hover trim for a takeoff, where the skids hold the airframe level).
    void engage(const HelicopterTrimResult& trim, f32 heading);

    // One step: the controls to fly. Throttle and engine are passed through
    // from the helicopter's current controls.
    HelicopterControls update(const Helicopter& helicopter, f32 step);

private:
    HelicopterAutopilotGains mGains;
    Horizontal mHorizontal = Horizontal::Position;
    Vertical mVertical = Vertical::Altitude;
    Math::vec3 mTargetPosition{0.0f};
    Math::vec3 mTargetVelocity{0.0f};
    f32 mTargetHeading = 0.0f;
    f32 mTargetVerticalSpeed = 0.0f;

    Math::vec3 mVelocityIntegral{0.0f};
    f32 mPitchTrim = 0.0f;
    f32 mRollTrim = 0.0f;
    f32 mPedalTrim = 0.0f;
    f32 mCollectiveTrim = 0.5f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ROTORCRAFT_HELICOPTERAUTOPILOT_H
