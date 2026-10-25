#include "marine/ScrewPropeller.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <kinetix/core/Algorithm.h>

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kDegrees = 3.14159265358979f / 180.0f;
} // namespace

ScrewPropeller::ScrewPropeller(const Math::vec3& hub, const Math::vec3& shaft, const Math::vec3& top,
                               f32 maxThrust)
    : mHub(hub), mShaft(shaft), mTop(top), mMaxThrust(maxThrust)
{
    reset();
}

void ScrewPropeller::updateForces(bool emitEffects)
{
    mForce = Math::vec3(0.0f);
    mDepth = -1.0f;
    if (!mBody || !mWater)
        return;

    const Math::vec3 hub = mBody->pointToWorld(mHub);
    const f32 depth = mWater->height(hub.x, hub.z) - hub.y;
    mDepth = depth;
    if (depth < 0.0f)
        return;

    // The thrust runs from the hub towards the shaft point, the other way in
    // reverse, turned by the rudder about the line down from the top point.
    Math::vec3 direction = Math::normalize(mBody->directionToWorld(mShaft - mHub));
    if (mReverse)
        direction = -direction;
    const Math::vec3 rudderAxis = Math::normalize(mBody->directionToWorld(mHub - mTop));
    const Math::vec3 force =
        (Math::angleAxis(mRudder * kDegrees, rudderAxis) * direction) * (mThrottle * mMaxThrust);
    mBody->addForceAtPoint(force, hub);
    mForce = force;

    // The wash: splash and ripple at the hub, harder with the screw near
    // the surface, once the throttle is open.
    if (emitEffects && mEffects && mThrottle > 0.1f)
    {
        const Math::vec3 unit = force * (1.0f / mMaxThrust);
        mEffects->splash(hub, unit * (depth < 0.2f ? 10.0f : 5.0f));
        mEffects->ripple(hub, unit * 10.0f);
    }
}

void ScrewPropeller::setThrottle(f32 value)
{
    if (!isfinite(value))
        value = 0.0f;
    value = Clamp(value, -1.0f, 1.0f);
    mThrottle = fabsf(value);
    mReverse = value < 0.0f;
}

void ScrewPropeller::setRudder(f32 value)
{
    if (!isfinite(value))
        value = 0.0f;
    mRudder = Clamp(value, -1.0f, 1.0f) * mMaxRudder;
}

void ScrewPropeller::setMaxRudderDegrees(f32 degrees)
{
    if (degrees > 0.0f && isfinite(degrees))
        mMaxRudder = degrees;
}

void ScrewPropeller::setMaxThrust(f32 thrust)
{
    if (thrust >= 0.0f && isfinite(thrust))
        mMaxThrust = thrust;
}

void ScrewPropeller::reset()
{
    setThrottle(0.0f);
    mRudder = 0.0f;
    mReverse = false;
}

void ScrewPropeller::toggleReverse()
{
    mThrottle = 0.0f;
    mReverse = !mReverse;
}

} // namespace kinetix::Physics
