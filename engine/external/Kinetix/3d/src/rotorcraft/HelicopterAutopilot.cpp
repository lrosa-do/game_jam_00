#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "rotorcraft/HelicopterAutopilot.h"

#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kGravity = 9.81f;

f32 wrapAngle(f32 angle)
{
    while (angle > kPi)
        angle -= 2.0f * kPi;
    while (angle < -kPi)
        angle += 2.0f * kPi;
    return angle;
}

Math::vec3 horizontal(const Math::vec3& v)
{
    return Math::vec3(v.x, 0.0f, v.z);
}

Math::vec3 clampLength(const Math::vec3& v, f32 limit)
{
    const f32 length = Math::length(v);
    return length > limit && length > 0.0f ? v * (limit / length) : v;
}
} // namespace

void HelicopterAutopilot::holdPosition(const Math::vec3& worldPosition, f32 heading)
{
    mHorizontal = Horizontal::Position;
    mTargetPosition.x = worldPosition.x;
    mTargetPosition.z = worldPosition.z;
    mTargetHeading = isfinite(heading) ? heading : 0.0f;
    if (mVertical == Vertical::Altitude)
        mTargetPosition.y = worldPosition.y;
}

void HelicopterAutopilot::holdVelocity(const Math::vec3& groundVelocity, f32 heading)
{
    mHorizontal = Horizontal::Velocity;
    mTargetVelocity = horizontal(groundVelocity);
    mTargetHeading = isfinite(heading) ? heading : 0.0f;
}

void HelicopterAutopilot::holdAltitude(f32 worldHeight)
{
    mVertical = Vertical::Altitude;
    mTargetPosition.y = worldHeight;
}

void HelicopterAutopilot::holdVerticalSpeed(f32 verticalSpeed)
{
    mVertical = Vertical::VerticalSpeed;
    mTargetVerticalSpeed = isfinite(verticalSpeed) ? verticalSpeed : 0.0f;
}

void HelicopterAutopilot::engage(const Helicopter& helicopter)
{
    const HelicopterControls& c = helicopter.controls();
    const HelicopterTelemetry& t = helicopter.telemetry();
    mPitchTrim = c.cyclicLongitudinal;
    mRollTrim = c.cyclicLateral;
    mPedalTrim = c.pedals;
    mCollectiveTrim = c.collective;
    const f32 heading = t.heading;
    const Math::vec3 forward(-sinf(heading), 0.0f, cosf(heading));
    const Math::vec3 right = Math::cross(forward, Math::vec3(0.0f, 1.0f, 0.0f));
    mVelocityIntegral = forward * (-kGravity * tanf(t.pitch)) + right * (kGravity * tanf(t.bank));
}

void HelicopterAutopilot::engage(const HelicopterTrimResult& trim, f32 heading)
{
    mPitchTrim = trim.cyclicLongitudinal;
    mRollTrim = trim.cyclicLateral;
    mPedalTrim = trim.pedals;
    mCollectiveTrim = trim.collective;
    const Math::vec3 forward(-sinf(heading), 0.0f, cosf(heading));
    const Math::vec3 right = Math::cross(forward, Math::vec3(0.0f, 1.0f, 0.0f));
    mVelocityIntegral = forward * (-kGravity * tanf(trim.pitch)) + right * (kGravity * tanf(trim.bank));
}

HelicopterControls HelicopterAutopilot::update(const Helicopter& helicopter, f32 step)
{
    const HelicopterAutopilotGains& g = mGains;
    const HelicopterTelemetry& t = helicopter.telemetry();
    HelicopterControls c = helicopter.controls();
    const RigidBody* body = helicopter.body();
    if (!body || !(step > 0.0f) || !isfinite(step))
        return c;
    // On the skids the attitude cannot follow the commands; the integrators would only wind up.
    const bool integrate = !t.onGround;

    const Math::vec3 position = body->position();
    const Math::vec3 velocity = horizontal(body->velocity());
    const Math::vec3 forward(-sinf(t.heading), 0.0f, cosf(t.heading));
    const Math::vec3 right = Math::cross(forward, Math::vec3(0.0f, 1.0f, 0.0f));

    Math::vec3 wanted = mTargetVelocity;
    if (mHorizontal == Horizontal::Position)
    {
        // Linear close in, never faster than can be braked to a stop over the remaining distance
        // at a third of the acceleration limit (square-root approach).
        const Math::vec3 offset = horizontal(mTargetPosition - position);
        const f32 distance = Math::length(offset);
        const f32 stopping = sqrtf(2.0f * (g.maxAcceleration / 3.0f) * distance);
        wanted = clampLength(offset * g.position, Min(g.maxSpeed, stopping));
    }
    // The integrator is for the trim attitude (steady state): it winds up only once the velocity
    // is nearly where wanted, so a long acceleration does not charge it against the stop that
    // follows; unwinding is always allowed.
    const Math::vec3 error = wanted - velocity;
    const bool unwinding = Math::dot(error, mVelocityIntegral) < 0.0f;
    if (integrate && (Math::length(error) < g.integrationWindow || unwinding))
        mVelocityIntegral = clampLength(mVelocityIntegral + error * (g.velocityIntegral * step), g.maxAcceleration);
    const Math::vec3 acceleration = clampLength(error * g.velocity + mVelocityIntegral, g.maxAcceleration);

    // Thrust tilted forward accelerates forward: nose down for forward acceleration, right side down for rightward.
    const f32 pitchCommand = Clamp(-atanf(Math::dot(acceleration, forward) / kGravity), -g.maxTilt, g.maxTilt);
    const f32 bankCommand = Clamp(atanf(Math::dot(acceleration, right) / kGravity), -g.maxTilt, g.maxTilt);

    const f32 pitchError = pitchCommand - t.pitch;
    const f32 bankError = bankCommand - t.bank;
    if (integrate)
    {
        mPitchTrim = Clamp(mPitchTrim - g.attitudeIntegral * pitchError * step, -1.0f, 1.0f);
        mRollTrim = Clamp(mRollTrim + g.attitudeIntegral * bankError * step, -1.0f, 1.0f);
    }
    // Stick forward pitches the nose down; a nose-up rate wants stick forward. Stick right rolls right.
    c.cyclicLongitudinal = Clamp(mPitchTrim - g.attitude * pitchError + g.pitchRate * t.bodyRates.y, -1.0f, 1.0f);
    c.cyclicLateral = Clamp(mRollTrim + g.attitude * bankError - g.rollRate * t.bodyRates.x, -1.0f, 1.0f);

    // Heading to a yaw rate, capped so a large change is a steady turn, not a kick of full pedal;
    // the integrator finds the anti-torque trim.
    const f32 headingError = wrapAngle(mTargetHeading - t.heading);
    if (integrate)
        mPedalTrim = Clamp(mPedalTrim + g.headingIntegral * Clamp(headingError, -0.2f, 0.2f) * step, -1.0f, 1.0f);
    const f32 yawRateCommand = Clamp(g.heading * headingError, -g.maxYawRate, g.maxYawRate);
    c.pedals = Clamp(mPedalTrim + g.yawRate * (yawRateCommand - t.bodyRates.z), -1.0f, 1.0f);

    f32 climb = mTargetVerticalSpeed;
    if (mVertical == Vertical::Altitude)
        climb = Clamp((mTargetPosition.y - position.y) * g.height, -g.maxClimb, g.maxClimb);
    const f32 climbError = climb - body->velocity().y;
    // A drooping rotor means the collective asks for more power than there is: lower it, and stop the integrator asking for more.
    const f32 droop = Max(g.rotorSpeedFloor - t.rotorSpeedPercent, 0.0f);
    if (droop <= 0.0f || climbError < 0.0f)
        mCollectiveTrim = Clamp(mCollectiveTrim + g.verticalIntegral * climbError * step, 0.0f, 1.0f);
    c.collective = Clamp(mCollectiveTrim + g.verticalSpeed * climbError - g.rotorSpeedRelief * droop, 0.0f, 1.0f);
    return c;
}

} // namespace kinetix::Physics
