#include "prisma3d/scene/VehicleDriver3D.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CarDamage3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Path3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

} // namespace

VehicleDriver3D::VehicleDriver3D()
        : Node(),
          mPathNode(),
          mPath(),
          mSpacing(2.0f),
          mLoop(false),
          mLoaded(false),
          mIndex(-1),
          mDirection(1),
          mProgress(0.0f),
          mCornering(7.0f),
          mTopSpeed(33.0f),
          mLaneOffset(0.0f),
          mStuck(0.0f),
          mBacking(0.0f),
          mActive(true),
          mBoost(false)
{
    setPhysicsProcess(true);
}

VehicleDriver3D::~VehicleDriver3D() = default;

void VehicleDriver3D::setPathNode(const ct::String& path)
{
    mPathNode = path;
    mLoaded = false;
    mIndex = -1;
}

const ct::String& VehicleDriver3D::getPathNode() const
{
    return mPathNode;
}

void VehicleDriver3D::setCornering(float cornering)
{
    mCornering = cornering > 0.1f ? cornering : 0.1f;
}

float VehicleDriver3D::getCornering() const
{
    return mCornering;
}

void VehicleDriver3D::setTopSpeed(float speed)
{
    mTopSpeed = speed > 1.0f ? speed : 1.0f;
}

float VehicleDriver3D::getTopSpeed() const
{
    return mTopSpeed;
}

void VehicleDriver3D::setLaneOffset(float offset)
{
    mLaneOffset = offset;
    mLoaded = false;
    mIndex = -1;
}

float VehicleDriver3D::getLaneOffset() const
{
    return mLaneOffset;
}

void VehicleDriver3D::setActive(bool active)
{
    if (active && !mActive) mIndex = -1;
    mActive = active;
    mBoost = false;
}

bool VehicleDriver3D::isActive() const
{
    return mActive;
}

float VehicleDriver3D::getProgress() const
{
    return mProgress;
}

bool VehicleDriver3D::wantsBoost() const
{
    return mBoost;
}

GroundVehicle3D* VehicleDriver3D::getVehicle() const
{
    Node* parent = getParent();
    return parent ? parent->castTo<GroundVehicle3D>() : nullptr;
}

bool VehicleDriver3D::loadPath()
{
    mLoaded = true;
    mPath.clear();
    Node* node = mPathNode.empty() ? nullptr : getNode(mPathNode);
    Path3D* path = node ? node->castTo<Path3D>() : nullptr;
    if (!path) return false;
    const Curve3D& curve = path->getCurve();
    const ct::Vector<Math::Vec3>& baked = curve.getBakedPoints();
    const Math::Mat4 place = path->getGlobalTransform();
    for (std::size_t i = 0; i < baked.size(); ++i) mPath.push_back(place.TransformPoint(baked[i]));
    mSpacing = curve.getBakeInterval() > 0.1f ? curve.getBakeInterval() : 0.1f;
    mLoop = curve.isClosed();
    return mPath.size() >= 8;
}

int VehicleDriver3D::wrap(int index) const
{
    const int count = static_cast<int>(mPath.size());
    if (!mLoop) return index < 0 ? 0 : (index >= count ? count - 1 : index);
    index %= count;
    return index < 0 ? index + count : index;
}

const Math::Vec3& VehicleDriver3D::point(int index) const
{
    return mPath[static_cast<std::size_t>(wrap(index))];
}

int VehicleDriver3D::steps(float metres) const
{
    const int count = static_cast<int>(metres / mSpacing);
    return count < 1 ? 1 : count;
}

int VehicleDriver3D::nearest(const Math::Vec3& place) const
{
    int best = 0;
    float bestDistance = 1.0e30f;
    for (std::size_t i = 0; i < mPath.size(); ++i)
    {
        const float distance = (mPath[i] - place).Length();
        if (distance >= bestDistance) continue;
        bestDistance = distance;
        best = static_cast<int>(i);
    }
    return best;
}

float VehicleDriver3D::follow(const Math::Vec3& place, const Math::Vec3& ahead)
{
    if (mIndex < 0)
    {
        if (!loadPath()) return 0.0f;
        mIndex = nearest(place);
        const Math::Vec3 along = point(mIndex + 1) - point(mIndex);
        mDirection = along.Dot(ahead) >= 0.0f ? 1 : -1;
        if (mLaneOffset != 0.0f)
        {
            ct::Vector<Math::Vec3> lane;
            const int count = static_cast<int>(mPath.size());
            for (int i = 0; i < count; ++i)
            {
                Math::Vec3 direction =
                        (point(i + 1) - point(i - 1)) * static_cast<float>(mDirection);
                direction.y = 0.0f;
                direction = direction.NormalizedSafe();
                const Math::Vec3 right(-direction.z, 0.0f, direction.x);
                lane.push_back(mPath[static_cast<std::size_t>(i)] + right * mLaneOffset);
            }
            mPath = lane;
        }
    }
    if (mPath.size() < 8) return 0.0f;

    int best = mIndex;
    float bestDistance = 1.0e30f;
    for (int k = -4; k <= 40; ++k)
    {
        const int index = mIndex + k * mDirection;
        const float distance = (point(index) - place).Length();
        if (distance >= bestDistance) continue;
        bestDistance = distance;
        best = index;
    }
    mProgress += static_cast<float>((best - mIndex) * mDirection);
    mIndex = wrap(best);
    return bestDistance;
}

void VehicleDriver3D::putOnRoad(GroundVehicle3D& vehicle, const Math::Vec3& at,
        const Math::Vec3& along) const
{
    vehicle.placeOnGround(at, along);
    for (std::size_t i = 0; i < vehicle.getChildCount(); ++i)
        if (CarDamage3D* damage = vehicle.getChild(i)->castTo<CarDamage3D>()) damage->reset();
}

void VehicleDriver3D::drive(GroundVehicle3D& vehicle, float step)
{
    mBoost = false;
    const Math::Mat4 frame = vehicle.getGlobalTransform();
    const Math::Vec3 place(frame.col3.x, frame.col3.y, frame.col3.z);
    const Math::Vec3 ahead = Math::Vec3(frame.col2.x, frame.col2.y, frame.col2.z).NormalizedSafe();
    const float speed = vehicle.getForwardSpeed();
    const float bestDistance = follow(place, ahead);
    if (mPath.size() < 8) return;

    if (bestDistance > 45.0f || mStuck > 6.0f)
    {
        putOnRoad(vehicle, point(mIndex), point(mIndex + 3 * mDirection) - point(mIndex));
        mStuck = 0.0f;
        mBacking = 0.0f;
        vehicle.setThrottle(0.0f);
        vehicle.setBrake(0.0f);
        vehicle.setSteering(0.0f);
        return;
    }

    const float reach = 7.0f + fabsf(speed) * 0.45f;
    const Math::Vec3 aim = point(mIndex + steps(reach) * mDirection);
    Math::Vec3 toAim = aim - place;
    toAim.y = 0.0f;
    const Math::Vec3 flat(ahead.x, 0.0f, ahead.z);
    const float turn = atan2f(flat.Cross(toAim).y, flat.Dot(toAim));
    float steer = clampf(-turn * 2.2f, -1.0f, 1.0f);

    const float look = 12.0f + fabsf(speed) * 1.6f;
    float sharpest = 0.0f;
    const int span = steps(10.0f);
    for (float d = 0.0f; d < look; d += 6.0f)
    {
        const int at = mIndex + steps(d) * mDirection;
        Math::Vec3 a = point(at + span * mDirection) - point(at);
        Math::Vec3 b = point(at + 2 * span * mDirection) - point(at + span * mDirection);
        a.y = 0.0f;
        b.y = 0.0f;
        const float bend = fabsf(atan2f(a.Cross(b).y, a.Dot(b))) / (a.Length() + 0.01f);
        const float weight = 1.0f - 0.45f * (d / look);
        if (bend * weight > sharpest) sharpest = bend * weight;
    }
    float allowed = sharpest > 1.0e-4f ? sqrtf(mCornering / sharpest) : mTopSpeed;
    allowed = clampf(allowed, 9.0f, mTopSpeed);

    if (fabsf(speed) < 1.2f) mStuck += step;
    else if (mBacking <= 0.0f)
        mStuck = 0.0f;
    if (mStuck > 1.6f && mBacking <= 0.0f) mBacking = 1.4f;
    float throttle = 0.0f;
    float brake = 0.0f;
    if (mBacking > 0.0f)
    {
        mBacking -= step;
        vehicle.setThrottle(0.0f);
        vehicle.setBrake(1.0f);
        vehicle.setHandbrake(0.0f);
        vehicle.setSteering(-steer);
        return;
    }

    if (speed < allowed - 0.5f) throttle = clampf((allowed - speed) * 0.4f, 0.3f, 1.0f);
    else if (speed > allowed + 2.5f)
        brake = clampf((speed - allowed) * 0.12f, 0.2f, 1.0f);
    mBoost = sharpest < 0.006f && fabsf(turn) < 0.12f && speed < allowed - 6.0f;
    vehicle.setThrottle(throttle);
    vehicle.setBrake(brake);
    vehicle.setHandbrake(0.0f);
    vehicle.setSteering(steer);
}

void VehicleDriver3D::notificationHandler(int what)
{
    Node::notificationHandler(what);
    if (what != kNotificationPhysicsProcess || !mActive) return;
    GroundVehicle3D* vehicle = getVehicle();
    if (!vehicle || !vehicle->isSimulated()) return;
    drive(*vehicle, static_cast<float>(getPhysicsProcessDeltaTime()));
}

void VehicleDriver3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = VehicleDriver3D;
    DB::addProperty<P>(PropertyInfo::nodePath("path", "Path3D"), &P::setPathNode, &P::getPathNode);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "cornering", 0.1, 50.0, 0.1, true),
            &P::setCornering, &P::getCornering);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "top_speed", 1.0, 200.0, 0.5, true),
            &P::setTopSpeed, &P::getTopSpeed);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "lane_offset", -20.0, 20.0, 0.1),
            &P::setLaneOffset, &P::getLaneOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "active"), &P::setActive, &P::isActive);
}

} // namespace prisma3d
