#ifndef KINETIX_PHYSICS_MARINE_SCREWPROPELLER_H
#define KINETIX_PHYSICS_MARINE_SCREWPROPELLER_H

#include "marine/WaterEffects.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// A screw propeller as a steerable push: three points of the body place it.
// `hub` is where the force acts, and whether it is under the surface says
// whether the screw bites; `shaft` is a point behind the hub along the
// shaft, towards which the thrust runs; `top` is a point above the hub, and
// the line from it down through the hub is the axis the rudder turns the
// thrust about, up to 45 degrees unless told otherwise. The thrust is throttle times the maximum,
// in newtons, and nothing acts with the hub out of the water.
class ScrewPropeller
{
public:
    ScrewPropeller() = default;
    ScrewPropeller(const Math::vec3& hub, const Math::vec3& shaft, const Math::vec3& top, f32 maxThrust);

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    void setWater(const WaterSurface* water)
    {
        mWater = water;
    }
    void setEffects(WaterEffects* effects)
    {
        mEffects = effects;
    }
    const Math::vec3& hub() const
    {
        return mHub;
    }

    // One step; with `emitEffects` the wash goes to the effects.
    void updateForces(bool emitEffects);

    // The size of `value` is the throttle, its sign the direction; the
    // rudder is -1..1 for -45..45 degrees.
    void setThrottle(f32 value);
    void setRudder(f32 value);
    f32 throttle() const
    {
        return mReverse ? -mThrottle : mThrottle;
    }
    f32 rudder() const
    {
        return mRudder / mMaxRudder;
    }
    f32 rudderDegrees() const
    {
        return mRudder;
    }
    // How far the rudder turns the thrust at -1 and 1; 45 degrees unless set.
    void setMaxRudderDegrees(f32 degrees);
    f32 maxRudderDegrees() const
    {
        return mMaxRudder;
    }
    f32 maxThrust() const
    {
        return mMaxThrust;
    }
    void setMaxThrust(f32 thrust);
    bool reverse() const
    {
        return mReverse;
    }
    void reset();
    void toggleReverse();
    // The engine as a gauge would show it, 50 to 100 percent with the
    // throttle.
    f32 powerPercent() const
    {
        return (0.5f + mThrottle / 2.0f) * 100.0f;
    }

    // The last step: the force applied, zero out of the water.
    const Math::vec3& force() const
    {
        return mForce;
    }
    f32 thrust() const
    {
        return Math::length(mForce);
    }
    bool inWater() const
    {
        return mDepth >= 0.0f;
    }

private:
    RigidBody* mBody = nullptr;
    const WaterSurface* mWater = nullptr;
    WaterEffects* mEffects = nullptr;
    Math::vec3 mHub{0.0f};
    Math::vec3 mShaft{0.0f, 0.0f, 1.0f};
    Math::vec3 mTop{0.0f, 1.0f, 0.0f};
    bool mReverse = false;
    f32 mMaxThrust = 0.0f;
    f32 mRudder = 0.0f; // degrees
    f32 mMaxRudder = 45.0f;
    f32 mThrottle = 0.0f;
    Math::vec3 mForce{0.0f};
    f32 mDepth = -1.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_SCREWPROPELLER_H
