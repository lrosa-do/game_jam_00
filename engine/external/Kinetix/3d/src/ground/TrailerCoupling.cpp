#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/TrailerCoupling.h"

#include "dynamics/ConeTwistJoint.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"
#include "ground/GroundVehicle.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
}

TrailerCoupling::~TrailerCoupling()
{
    detach();
}

void TrailerCoupling::placeTrailer(const RigidBody& tow, RigidBody& trailer, const CouplingParameters& parameters)
{
    const Math::vec3 point = tow.pointToWorld(parameters.towPoint);
    trailer.setOrientation(tow.orientation());
    trailer.setPosition(point - tow.orientation() * parameters.trailerPoint);
    trailer.setVelocity(tow.velocity());
    trailer.setAngularVelocity(Math::vec3(0.0f));
    trailer.clearAccumulators();
    trailer.calculateDerivedData();
}

bool TrailerCoupling::attach(PhysicsWorld& world, GroundVehicle& tow, GroundVehicle& trailer,
                             const CouplingParameters& parameters)
{
    detach();
    if (!tow.body() || !trailer.body() || tow.body() == trailer.body())
    {
        Log::error("TrailerCoupling: both vehicles need a body of their own");
        return false;
    }
    RigidBody& a = *tow.body();
    RigidBody& b = *trailer.body();
    mParameters = parameters;
    if (parameters.type == CouplingType::FifthWheel)
    {
        // Twist about the common vertical (yaw articulation); the swing cone is pitch about the
        // constraint frame's Y (the right axis here) and roll about its Z (forward).
        const Math::vec3 up(0.0f, 1.0f, 0.0f);
        const Math::vec3 forward(0.0f, 0.0f, 1.0f);
        mFifthWheel = new ConeTwistJoint(a, parameters.towPoint, up, forward, b, parameters.trailerPoint,
                                         b.directionToLocal(a.directionToWorld(up)),
                                         b.directionToLocal(a.directionToWorld(forward)));
        mFifthWheel->setTwistLimits(-fabsf(parameters.maxArticulation), fabsf(parameters.maxArticulation));
        mFifthWheel->setSwingLimits(fabsf(parameters.maxPitch), fabsf(parameters.maxRoll));
        mJoint = mFifthWheel;
    }
    else
    {
        mBall = new PointJoint(a, parameters.towPoint, b, parameters.trailerPoint);
        mJoint = mBall;
    }
    world.addJoint(mJoint);
    world.setPairIgnored(tow.bodyId(), trailer.bodyId(), true);
    if (parameters.brakeLink)
        trailer.setBrakeSource(&tow);
    mWorld = &world;
    mTow = &tow;
    mTrailer = &trailer;
    return true;
}

void TrailerCoupling::detach()
{
    if (!mJoint)
        return;
    if (mWorld)
    {
        mWorld->removeJoint(mJoint);
        if (mTow && mTrailer)
            mWorld->setPairIgnored(mTow->bodyId(), mTrailer->bodyId(), false);
    }
    if (mTrailer && mTrailer->brakeSource() == mTow)
        mTrailer->setBrakeSource(nullptr);
    delete mJoint;
    mJoint = nullptr;
    mBall = nullptr;
    mFifthWheel = nullptr;
    mWorld = nullptr;
    mTow = nullptr;
    mTrailer = nullptr;
}

Math::vec3 TrailerCoupling::towPointWorld() const
{
    if (!mTow || !mTow->body())
        return Math::vec3(0.0f);
    return mTow->body()->pointToWorld(mParameters.towPoint);
}

Math::vec3 TrailerCoupling::trailerPointWorld() const
{
    if (!mTrailer || !mTrailer->body())
        return Math::vec3(0.0f);
    return mTrailer->body()->pointToWorld(mParameters.trailerPoint);
}

f32 TrailerCoupling::separation() const
{
    if (!attached())
        return 0.0f;
    return Math::length(towPointWorld() - trailerPointWorld());
}

f32 TrailerCoupling::articulationAngle() const
{
    if (!attached())
        return 0.0f;
    const RigidBody& a = *mTow->body();
    const RigidBody& b = *mTrailer->body();
    // Trailer heading in the tow's frame projected on its ground plane; positive turned towards
    // the tow's right (-X).
    const Math::vec3 local = a.directionToLocal(b.directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f)));
    f32 angle = atan2f(-local.x, local.z);
    if (angle > kPi)
        angle -= 2.0f * kPi;
    if (angle < -kPi)
        angle += 2.0f * kPi;
    return angle;
}

} // namespace kinetix::Physics
