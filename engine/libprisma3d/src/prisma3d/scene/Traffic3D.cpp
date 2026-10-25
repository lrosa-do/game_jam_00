#include "prisma3d/scene/Traffic3D.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CarDamage3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/VehicleDriver3D.h"

#include <math.h>

namespace prisma3d
{

Traffic3D::Traffic3D()
        : Node3D(),
          mPathNode(),
          mTarget(),
          mCars(),
          mAheadDistance(350.0f),
          mBehindDistance(150.0f),
          mRailLayer(4u),
          mRailMask(8u),
          mSpread(12345u),
          mGathered(false)
{
    setPhysicsProcess(true);
}

Traffic3D::~Traffic3D() = default;

void Traffic3D::setPathNode(const ct::String& path)
{
    mPathNode = path;
    mGathered = false;
}

const ct::String& Traffic3D::getPathNode() const
{
    return mPathNode;
}

void Traffic3D::setTarget(const ct::String& path)
{
    mTarget = path;
}

const ct::String& Traffic3D::getTarget() const
{
    return mTarget;
}

void Traffic3D::setAheadDistance(float distance)
{
    mAheadDistance = distance > 0.0f ? distance : 0.0f;
}

float Traffic3D::getAheadDistance() const
{
    return mAheadDistance;
}

void Traffic3D::setBehindDistance(float distance)
{
    mBehindDistance = distance > 0.0f ? distance : 0.0f;
}

float Traffic3D::getBehindDistance() const
{
    return mBehindDistance;
}

void Traffic3D::setRailLayer(std::uint32_t layer)
{
    mRailLayer = layer;
}

std::uint32_t Traffic3D::getRailLayer() const
{
    return mRailLayer;
}

void Traffic3D::setRailMask(std::uint32_t mask)
{
    mRailMask = mask;
}

std::uint32_t Traffic3D::getRailMask() const
{
    return mRailMask;
}

int Traffic3D::getActiveCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < mCars.size(); ++i)
        if (mCars[i].active) ++count;
    return count;
}

bool Traffic3D::gather()
{
    mGathered = true;
    mCars.clear();
    Node* node = mPathNode.empty() ? nullptr : getNode(mPathNode);
    const Path3D* path = node ? node->castTo<Path3D>() : nullptr;
    if (!path) return false;
    const Math::Mat4 toPath = path->getGlobalTransform().InverseAffine();
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        GroundVehicle3D* vehicle = getChild(i)->castTo<GroundVehicle3D>();
        if (!vehicle) continue;
        Car car;
        car.vehicle = vehicle;
        for (std::size_t c = 0; c < vehicle->getChildCount(); ++c)
        {
            Node* child = vehicle->getChild(c);
            if (!car.driver) car.driver = child->castTo<VehicleDriver3D>();
            if (!car.damage) car.damage = child->castTo<CarDamage3D>();
        }
        if (car.driver)
        {
            car.lane = car.driver->getLaneOffset();
            car.speed = car.driver->getTopSpeed();
        }
        car.layer = vehicle->getCollisionLayer();
        car.mask = vehicle->getCollisionMask();
        car.offset = path->getCurve().getClosestOffset(
                toPath.TransformPoint(vehicle->getGlobalPosition()));
        mCars.push_back(car);
    }
    for (std::size_t i = 0; i < mCars.size(); ++i)
    {
        setActive(mCars[i], false, *path);
        putOnLane(mCars[i], *path);
    }
    return true;
}

Math::Vec3 Traffic3D::laneDirection(const Path3D& path, float offset) const
{
    const Curve3D& curve = path.getCurve();
    const Math::Mat4 place = path.getGlobalTransform();
    Math::Vec3 direction = place.TransformVector(
            curve.sampleBaked(offset + 2.0f) - curve.sampleBaked(offset - 2.0f));
    direction.y = 0.0f;
    return direction.NormalizedSafe();
}

void Traffic3D::putOnLane(Car& car, const Path3D& path) const
{
    const Math::Vec3 forward = laneDirection(path, car.offset);
    const Math::Vec3 right(-forward.z, 0.0f, forward.x);
    Math::Vec3 at =
            path.getGlobalTransform().TransformPoint(path.getCurve().sampleBaked(car.offset));
    at = at + right * car.lane;
    at.y += car.vehicle->getRestHeight();
    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    const Math::Vec3 left = up.Cross(forward);
    Math::Mat4 frame = Math::Mat4::Identity();
    frame.col0 = Math::Vec4(left.x, left.y, left.z, 0.0f);
    frame.col1 = Math::Vec4(0.0f, 1.0f, 0.0f, 0.0f);
    frame.col2 = Math::Vec4(forward.x, forward.y, forward.z, 0.0f);
    frame.col3 = Math::Vec4(at.x, at.y, at.z, 1.0f);
    car.vehicle->setGlobalTransform(frame);
}

void Traffic3D::setActive(Car& car, bool active, const Path3D& path)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (car.active == active || !server) return;
    car.active = active;
    const PhysicsBodyId body = car.vehicle->getBody();
    if (car.driver) car.driver->setActive(false);
    car.vehicle->setThrottle(0.0f);
    car.vehicle->setBrake(0.0f);
    car.vehicle->setSteering(0.0f);
    if (active)
    {
        const Math::Vec3 carried = laneDirection(path, car.offset) * car.speed;
        car.vehicle->setCollisionLayer(car.layer);
        car.vehicle->setCollisionMask(car.mask);
        server->bodySetMode(body, PhysicsBodyMode::Rigid);
        server->bodySetLinearVelocity(body, carried);
        return;
    }
    server->bodySetMode(body, PhysicsBodyMode::Kinematic);
    car.vehicle->setCollisionLayer(mRailLayer);
    car.vehicle->setCollisionMask(mRailMask);
}

void Traffic3D::update(float step)
{
    if (!mGathered && !gather()) return;
    Node* node = mPathNode.empty() ? nullptr : getNode(mPathNode);
    const Path3D* path = node ? node->castTo<Path3D>() : nullptr;
    Node* targetNode = mTarget.empty() ? nullptr : getNode(mTarget);
    const Node3D* target = targetNode ? targetNode->castTo<Node3D>() : nullptr;
    if (!path || !target) return;
    const Curve3D& curve = path->getCurve();
    const float length = curve.getBakedLength();
    if (length <= 0.0f) return;
    const Math::Mat4 toPath = path->getGlobalTransform().InverseAffine();
    const float targetOffset =
            curve.getClosestOffset(toPath.TransformPoint(target->getGlobalPosition()));

    for (std::size_t i = 0; i < mCars.size(); ++i)
    {
        Car& car = mCars[i];
        if (!car.vehicle) continue;
        if (car.active)
            car.offset =
                    curve.getClosestOffset(toPath.TransformPoint(car.vehicle->getGlobalPosition()));
        else if (car.damage && car.damage->getStepImpulse() > 0.0f)
            setActive(car, true, *path);
        else
        {
            car.offset += car.speed * step;
            if (car.offset < length - 1.0f) putOnLane(car, *path);
        }

        if (car.offset >= targetOffset - mBehindDistance && car.offset < length - 1.0f) continue;
        mSpread = mSpread * 1664525u + 1013904223u;
        const float spread = static_cast<float>(mSpread >> 8) / 16777216.0f * 80.0f;
        const float offset = targetOffset + mAheadDistance + spread;
        if (offset > length - 5.0f) continue;
        setActive(car, false, *path);
        car.offset = offset;
        putOnLane(car, *path);
        if (car.damage) car.damage->reset();
    }
}

void Traffic3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    if (what == kNotificationPhysicsProcess)
        update(static_cast<float>(getPhysicsProcessDeltaTime()));
}

void Traffic3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Traffic3D;
    DB::addProperty<P>(PropertyInfo::nodePath("path", "Path3D"), &P::setPathNode, &P::getPathNode);
    DB::addProperty<P>(PropertyInfo::nodePath("target", "Node3D"), &P::setTarget, &P::getTarget);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "ahead_distance", 0.0, 5000.0, 1.0, true),
            &P::setAheadDistance, &P::getAheadDistance);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "behind_distance", 0.0, 5000.0, 1.0, true),
            &P::setBehindDistance, &P::getBehindDistance);
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("rail_layer"), &P::setRailLayer,
            &P::getRailLayer);
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("rail_mask"), &P::setRailMask,
            &P::getRailMask);
}

} // namespace prisma3d
