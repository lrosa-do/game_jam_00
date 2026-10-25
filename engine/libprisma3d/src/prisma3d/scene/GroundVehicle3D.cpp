#include "prisma3d/scene/GroundVehicle3D.h"

#include "NodeFactory.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SkidMarks3D.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

float clamp01(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

} // namespace

GroundVehicle3D::GroundVehicle3D()
        : RigidBody3D(),
          mVehicle(),
          mLayout(),
          mShape(),
          mControls(),
          mSkidMarksPath(),
          mCollisionMesh(),
          mCollisionScale(1.0f, 1.0f, 1.0f),
          mCollisionOffset(0.0f, 0.0f, 0.0f),
          mHullPoints(),
          mHullLoaded(false),
          mReverseHold(0.0f),
          mSkidSlot(-1),
          mRayMask(0xFFFFFFFFu),
          mWheelRadius(0.0f),
          mPower(1.0f),
          mTopSpeedKmh(0.0f),
          mGrip(1.0f),
          mRollInfluence(1.0f),
          mHandbrakeTorque(0.0f),
          mDrive(static_cast<int>(PhysicsDriveLayout::RearWheel)),
          mAutomatic(true),
          mAntiLock(false),
          mDirty(false),
          mParked(true),
          mReverse(false)
{
    setPhysicsProcess(true);
}

GroundVehicle3D::~GroundVehicle3D()
{
    release();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mShape.valid()) server->shapeFree(mShape);
}

void GroundVehicle3D::setDrive(int drive)
{
    mDrive = drive < 0 ? 0 : (drive > 2 ? 2 : drive);
    mDirty = true;
}

int GroundVehicle3D::getDrive() const
{
    return mDrive;
}

void GroundVehicle3D::setAutomatic(bool automatic)
{
    mAutomatic = automatic;
    mDirty = true;
}

bool GroundVehicle3D::isAutomatic() const
{
    return mAutomatic;
}

void GroundVehicle3D::setAntiLock(bool antiLock)
{
    mAntiLock = antiLock;
    mDirty = true;
}

bool GroundVehicle3D::isAntiLock() const
{
    return mAntiLock;
}

void GroundVehicle3D::setWheelRayMask(std::uint32_t mask)
{
    mRayMask = mask;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetRayMask(mVehicle, mRayMask);
}

std::uint32_t GroundVehicle3D::getWheelRayMask() const
{
    return mRayMask;
}

void GroundVehicle3D::setWheelRadius(float radius)
{
    mWheelRadius = radius < 0.0f ? 0.0f : radius;
    mDirty = true;
}

float GroundVehicle3D::getWheelRadiusSetting() const
{
    return mWheelRadius;
}

void GroundVehicle3D::setTopSpeedKmh(float kmh)
{
    mTopSpeedKmh = kmh < 0.0f ? 0.0f : kmh;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetTopSpeed(mVehicle, mTopSpeedKmh / 3.6f);
}

float GroundVehicle3D::getTopSpeedKmh() const
{
    return mTopSpeedKmh;
}

void GroundVehicle3D::setPower(float power)
{
    mPower = power < 0.01f ? 0.01f : power;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetPowerScale(mVehicle, mPower);
}

float GroundVehicle3D::getPower() const
{
    return mPower;
}

void GroundVehicle3D::setGrip(float grip)
{
    mGrip = grip < 0.01f ? 0.01f : grip;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetGrip(mVehicle, mGrip);
}

float GroundVehicle3D::getGrip() const
{
    return mGrip;
}

void GroundVehicle3D::setRollInfluence(float influence)
{
    mRollInfluence = influence < 0.0f ? 0.0f : (influence > 1.0f ? 1.0f : influence);
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetRollInfluence(mVehicle, mRollInfluence);
}

float GroundVehicle3D::getRollInfluence() const
{
    return mRollInfluence;
}

void GroundVehicle3D::setHandbrakeTorque(float torque)
{
    mHandbrakeTorque = torque < 0.0f ? 0.0f : torque;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid())
        server->groundVehicleSetHandbrakeTorque(mVehicle, mHandbrakeTorque);
}

float GroundVehicle3D::getHandbrakeTorque() const
{
    return mHandbrakeTorque;
}

void GroundVehicle3D::setAxleGrip(int axle, float scale)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleSetAxleGrip(mVehicle, axle, scale);
}

Node3D* GroundVehicle3D::wheelNode(int wheel) const
{
    if (wheel < 0 || wheel >= kWheels || mWheelPaths[wheel].empty()) return nullptr;
    Node* node = const_cast<GroundVehicle3D*>(this)->getNode(mWheelPaths[wheel]);
    return node ? node->castTo<Node3D>() : nullptr;
}

void GroundVehicle3D::fitAxles()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mVehicle.valid()) return;
    Node3D* wheels[kWheels];
    bool all = true;
    for (int i = 0; i < kWheels; ++i)
    {
        wheels[i] = wheelNode(i);
        all = all && wheels[i];
    }
    for (int axle = 0; axle < 2; ++axle)
    {
        PhysicsGroundWheelState state;
        if (!server->groundVehicleGetWheelState(mVehicle, axle * 2, state)) continue;
        float position = state.axlePosition;
        float halfTrack = state.halfTrack;
        if (all)
        {
            const Math::Vec4 left = wheels[axle * 2]->getTransform().col3;
            const Math::Vec4 right = wheels[axle * 2 + 1]->getTransform().col3;
            position = 0.5f * (left.z + right.z);
            halfTrack = 0.5f * (fabsf(left.x) + fabsf(right.x));
        }
        server->groundVehicleSetAxle(mVehicle, axle, position, halfTrack, mWheelRadius);
    }
    mLayout.restHeight = server->groundVehicleGetRestHeight(mVehicle);
    server->groundVehicleSetTopSpeed(mVehicle, mTopSpeedKmh / 3.6f);
}

void GroundVehicle3D::setThrottle(float throttle)
{
    mControls.throttle = clamp01(throttle);
}

float GroundVehicle3D::getThrottle() const
{
    return mControls.throttle;
}

void GroundVehicle3D::setBrake(float brake)
{
    mControls.brake = clamp01(brake);
}

float GroundVehicle3D::getBrake() const
{
    return mControls.brake;
}

void GroundVehicle3D::setHandbrake(float handbrake)
{
    mControls.handbrake = clamp01(handbrake);
}

float GroundVehicle3D::getHandbrake() const
{
    return mControls.handbrake;
}

void GroundVehicle3D::setSteering(float steering)
{
    mControls.steering = steering < -1.0f ? -1.0f : (steering > 1.0f ? 1.0f : steering);
}

float GroundVehicle3D::getSteering() const
{
    return mControls.steering;
}

void GroundVehicle3D::setWheelPath(int wheel, const ct::String& path)
{
    if (wheel >= 0 && wheel < kWheels) mWheelPaths[wheel] = path;
}

ct::String GroundVehicle3D::getWheelPath(int wheel) const
{
    return wheel >= 0 && wheel < kWheels ? mWheelPaths[wheel] : ct::String();
}

void GroundVehicle3D::setWheelFrontLeft(const ct::String& path)
{
    setWheelPath(0, path);
}

ct::String GroundVehicle3D::getWheelFrontLeft() const
{
    return mWheelPaths[0];
}

void GroundVehicle3D::setWheelFrontRight(const ct::String& path)
{
    setWheelPath(1, path);
}

ct::String GroundVehicle3D::getWheelFrontRight() const
{
    return mWheelPaths[1];
}

void GroundVehicle3D::setWheelRearLeft(const ct::String& path)
{
    setWheelPath(2, path);
}

ct::String GroundVehicle3D::getWheelRearLeft() const
{
    return mWheelPaths[2];
}

void GroundVehicle3D::setWheelRearRight(const ct::String& path)
{
    setWheelPath(3, path);
}

ct::String GroundVehicle3D::getWheelRearRight() const
{
    return mWheelPaths[3];
}

void GroundVehicle3D::setSkidMarksPath(const ct::String& path)
{
    mSkidMarksPath = path;
    mSkidSlot = -1;
}

const ct::String& GroundVehicle3D::getSkidMarksPath() const
{
    return mSkidMarksPath;
}

void GroundVehicle3D::setCollisionMesh(const ct::String& path)
{
    if (path == mCollisionMesh) return;
    mCollisionMesh = path;
    mHullLoaded = false;
    mDirty = true;
}

const ct::String& GroundVehicle3D::getCollisionMesh() const
{
    return mCollisionMesh;
}

void GroundVehicle3D::setCollisionScale(const Math::Vec3& scale)
{
    mCollisionScale = scale;
    mHullLoaded = false;
    mDirty = true;
}

const Math::Vec3& GroundVehicle3D::getCollisionScale() const
{
    return mCollisionScale;
}

void GroundVehicle3D::setCollisionOffset(const Math::Vec3& offset)
{
    mCollisionOffset = offset;
    mHullLoaded = false;
    mDirty = true;
}

const Math::Vec3& GroundVehicle3D::getCollisionOffset() const
{
    return mCollisionOffset;
}

void GroundVehicle3D::loadHull()
{
    mHullLoaded = true;
    mHullPoints.clear();
    AssetLoader* loader = Main::assets();
    if (!loader || mCollisionMesh.empty()) return;
    ModelId id = loader->findModelByPath(mCollisionMesh);
    if (!id.valid())
    {
        if (!FileSystem::getSingleton().exists(AssetLoader::modelFile(mCollisionMesh)))
        {
            log_error("ground vehicle: collision mesh '%s' was not found", mCollisionMesh.c_str());
            return;
        }
        id = loader->requestModel(mCollisionMesh);
    }
    if (!id.valid()) return;
    if (loader->modelStatus(id) == LoadStatus::InProgress) loader->waitAll();
    const ModelInfo* info = loader->model(id);
    if (!info || info->status != LoadStatus::Loaded || !info->importer)
    {
        log_error("ground vehicle: collision mesh '%s' did not load", mCollisionMesh.c_str());
        return;
    }
    const ModelData& document = info->document;
    const Math::Mat4 bake = Math::Mat4::Scale(Math::Vec3(info->scale, info->scale, info->scale));
    for (std::size_t n = 0; n < document.nodes.size(); ++n)
    {
        const ModelNodeData& node = document.nodes[n];
        if (node.mesh < 0 || static_cast<std::size_t>(node.mesh) >= document.meshes.size())
            continue;
        const Math::Mat4 place = bake * document.nodeGlobalTransform(static_cast<int>(n));
        const ModelMeshData& mesh = document.meshes[static_cast<std::size_t>(node.mesh)];
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
        {
            SurfaceData surface;
            if (!info->importer->readPrimitive(document, mesh.primitives[p], surface)) continue;
            for (std::size_t i = 0; i < surface.positions.size(); ++i)
            {
                const Math::Vec3 at = place.TransformPoint(surface.positions[i]);
                const Math::Vec3 point(at.x * mCollisionScale.x + mCollisionOffset.x,
                        at.y * mCollisionScale.y + mCollisionOffset.y,
                        at.z * mCollisionScale.z + mCollisionOffset.z);
                bool seen = false;
                for (std::size_t k = 0; k < mHullPoints.size() && !seen; ++k)
                    seen = mHullPoints[k].x == point.x && mHullPoints[k].y == point.y &&
                           mHullPoints[k].z == point.z;
                if (!seen) mHullPoints.push_back(point);
            }
        }
    }
    if (mHullPoints.size() < 4)
    {
        log_error("ground vehicle: collision mesh '%s' has too few points", mCollisionMesh.c_str());
        mHullPoints.clear();
    }
}

PhysicsShapeId GroundVehicle3D::makeHull(float rear, float front) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return PhysicsShapeId();
    ct::Vector<Math::Vec3> points;
    if (mHullPoints.empty())
    {
        const Math::Vec3 half = mLayout.collisionHalfExtents;
        for (int i = 0; i < 8; ++i)
            points.push_back(Math::Vec3((i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y,
                    (i & 4) ? front : rear));
    }
    else
    {
        for (std::size_t i = 0; i < mHullPoints.size(); ++i)
        {
            Math::Vec3 point = mHullPoints[i];
            point.z = point.z < rear ? rear : (point.z > front ? front : point.z);
            points.push_back(point);
        }
    }
    PhysicsShapeId hull = server->shapeCreate(PhysicsShapeType::ConvexHull);
    server->shapeSetConvexHull(hull, points.data(), points.size());
    return hull;
}

void GroundVehicle3D::updateSkids()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    Node* marksNode = mSkidMarksPath.empty() ? nullptr : getNode(mSkidMarksPath);
    SkidMarks3D* marks = marksNode ? marksNode->castTo<SkidMarks3D>() : nullptr;
    if (!server || !marks || !mVehicle.valid()) return;
    if (mSkidSlot < 0) mSkidSlot = marks->claimWheels(kWheels);
    if (mSkidSlot < 0) return;
    const Math::Mat4 frame = server->bodyGetTransform(getBody());
    const bool upright = frame.col1.y > 0.7f;
    const Math::Vec3 across =
            Math::Vec3(frame.col0.x, frame.col0.y, frame.col0.z).NormalizedSafe() * 0.14f;
    PhysicsGroundVehicleState vehicle;
    server->groundVehicleGetState(mVehicle, vehicle);
    const float speed = fabsf(vehicle.forwardSpeed);
    for (int w = 0; w < kWheels; ++w)
    {
        PhysicsGroundWheelState wheel;
        const bool known = server->groundVehicleGetWheelState(mVehicle, w, wheel);
        if (!known || !upright || !wheel.inContact || wheel.detached || speed < 0.5f)
        {
            marks->lift(mSkidSlot + w);
            continue;
        }
        const float dark = fminf(marks->getRollingStrength() + 0.75f * wheel.slide, 0.85f);
        if (dark < 0.05f)
        {
            marks->lift(mSkidSlot + w);
            continue;
        }
        marks->touch(mSkidSlot + w, wheel.contactPoint, across, dark, getBody());
    }
}

PhysicsGroundVehicleId GroundVehicle3D::getVehicle() const
{
    return mVehicle;
}

const PhysicsGroundVehicleLayout& GroundVehicle3D::getLayout() const
{
    return mLayout;
}

Math::Vec3 GroundVehicle3D::getCollisionHalfExtents() const
{
    return mLayout.collisionHalfExtents;
}

float GroundVehicle3D::getRestHeight() const
{
    return mLayout.restHeight;
}

float GroundVehicle3D::getSpeedKmh() const
{
    return getForwardSpeed() * 3.6f;
}

float GroundVehicle3D::getForwardSpeed() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundVehicleState state;
    return server && server->groundVehicleGetState(mVehicle, state) ? state.forwardSpeed : 0.0f;
}

float GroundVehicle3D::getRpm() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundVehicleState state;
    return server && server->groundVehicleGetState(mVehicle, state) ? state.engineRpm : 0.0f;
}

int GroundVehicle3D::getGear() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundVehicleState state;
    return server && server->groundVehicleGetState(mVehicle, state) ? state.gear : 0;
}

int GroundVehicle3D::getWheelCount() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->groundVehicleGetWheelCount(mVehicle) : 0;
}

bool GroundVehicle3D::isWheelInContact(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state) && state.inContact;
}

float GroundVehicle3D::getWheelSlipRatio(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state) ? state.slipRatio
                                                                                : 0.0f;
}

float GroundVehicle3D::getWheelSlide(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state) &&
                           state.inContact && !state.detached
                   ? state.slide
                   : 0.0f;
}

float GroundVehicle3D::getWheelSlipAngle(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state) ? state.slipAngle
                                                                                : 0.0f;
}

float GroundVehicle3D::getWheelRadius(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state)
                   ? state.radius * state.radiusScale
                   : 0.0f;
}

Math::Mat4 GroundVehicle3D::getWheelLocalTransform(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state)
                   ? state.local
                   : Math::Mat4::Identity();
}

bool GroundVehicle3D::isWheelDetached(int wheel) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsGroundWheelState state;
    return server && server->groundVehicleGetWheelState(mVehicle, wheel, state) && state.detached;
}

void GroundVehicle3D::setWheelDetached(int wheel, bool detached)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server) server->groundVehicleSetWheelDetached(mVehicle, wheel, detached);
}

void GroundVehicle3D::setWheelRadiusScale(int wheel, float scale)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server) server->groundVehicleSetWheelRadiusScale(mVehicle, wheel, scale);
}

void GroundVehicle3D::stopWheels()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server) server->groundVehicleStopWheels(mVehicle);
}

void GroundVehicle3D::placeOnGround(const Math::Vec3& at, const Math::Vec3& forward)
{
    Math::Vec3 ahead(forward.x, 0.0f, forward.z);
    ahead = ahead.NormalizedSafe();
    if (ahead.LengthSquared() < 0.5f) ahead = Math::Vec3(0.0f, 0.0f, 1.0f);
    Math::Vec3 place = at;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsRayParameters ray;
    ray.from = at + Math::Vec3(0.0f, 4.0f, 0.0f);
    ray.to = at - Math::Vec3(0.0f, 6.0f, 0.0f);
    ray.exclude = getBody();
    PhysicsRayResult hit;
    if (server && server->intersectRay(ray, hit)) place.y = hit.position.y;
    place.y += mLayout.restHeight + 0.03f;
    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    const Math::Vec3 left = up.Cross(ahead);
    Math::Mat4 frame = Math::Mat4::Identity();
    frame.col0 = Math::Vec4(left.x, left.y, left.z, 0.0f);
    frame.col1 = Math::Vec4(0.0f, 1.0f, 0.0f, 0.0f);
    frame.col2 = Math::Vec4(ahead.x, ahead.y, ahead.z, 0.0f);
    frame.col3 = Math::Vec4(place.x, place.y, place.z, 1.0f);
    setGlobalTransform(frame);
    setLinearVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    setAngularVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    stopWheels();
}

void GroundVehicle3D::setLengthwiseExtent(float rear, float front)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mShape.valid()) return;
    const PhysicsShapeId hull = makeHull(rear, front);
    server->bodySetShape(getBody(), hull, Math::Mat4::Identity());
    server->shapeFree(mShape);
    mShape = hull;
}

void GroundVehicle3D::bindShape(const CollisionShape3D* ignore)
{
    (void) ignore;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !getBody().valid()) return;
    if (!mShape.valid())
    {
        mShape = server->shapeCreate(PhysicsShapeType::Box);
        server->shapeSetBox(mShape, mLayout.collisionHalfExtents);
    }
    server->bodySetShape(getBody(), mShape, Math::Mat4::Identity());
}

void GroundVehicle3D::build()
{
    release();
    mDirty = false;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !isSimulated()) return;
    PhysicsPassengerCarSettings settings;
    settings.drive = static_cast<PhysicsDriveLayout>(mDrive);
    settings.automatic = mAutomatic;
    settings.antiLock = mAntiLock;
    mVehicle = server->groundVehicleCreatePassengerCar(getBody(), settings, mLayout);
    if (!server->groundVehicleValid(mVehicle))
    {
        mVehicle = PhysicsGroundVehicleId();
        return;
    }
    server->groundVehicleSetRayMask(mVehicle, mRayMask);
    server->groundVehicleSetPowerScale(mVehicle, mPower);
    server->groundVehicleSetGrip(mVehicle, mGrip);
    server->groundVehicleSetRollInfluence(mVehicle, mRollInfluence);
    server->groundVehicleSetHandbrakeTorque(mVehicle, mHandbrakeTorque);
    fitAxles();
    PhysicsShapeId box = server->shapeCreate(PhysicsShapeType::Box);
    server->shapeSetBox(box, mLayout.collisionHalfExtents);
    server->bodySetShape(getBody(), box, Math::Mat4::Identity());
    if (mShape.valid()) server->shapeFree(mShape);
    mShape = box;
    setFriction(0.5f);
    setMass(mLayout.mass);
    setInertia(server->bodyGetInertia(getBody()));
    if (!mHullLoaded) loadHull();
    if (!mHullPoints.empty())
    {
        const PhysicsShapeId hull = makeHull(-1.0e30f, 1.0e30f);
        server->bodySetShape(getBody(), hull, Math::Mat4::Identity());
        server->shapeFree(mShape);
        mShape = hull;
    }
    setCanSleep(false);
    placeWheels();
}

void GroundVehicle3D::release()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->groundVehicleFree(mVehicle);
    mVehicle = PhysicsGroundVehicleId();
}

void GroundVehicle3D::park()
{
    mParked = true;
    mReverse = false;
    mControls = PhysicsGroundVehicleControls();
    mControls.handbrake = 1.0f;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mVehicle.valid()) return;
    server->groundVehicleSelectGear(mVehicle, 0);
    server->groundVehicleSetControls(mVehicle, mControls);
}

bool GroundVehicle3D::isReversing() const
{
    return mReverse;
}

void GroundVehicle3D::pushControls()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mVehicle.valid()) return;
    const bool moving = mControls.throttle > 0.0f || mControls.brake > 0.0f;
    if (mParked && !moving)
    {
        PhysicsGroundVehicleControls hold = mControls;
        hold.handbrake = 1.0f;
        server->groundVehicleSetControls(mVehicle, hold);
        return;
    }
    if (mParked)
    {
        mParked = false;
        server->groundVehicleSelectGear(mVehicle, 1);
    }
    PhysicsGroundVehicleState state;
    server->groundVehicleGetState(mVehicle, state);
    const float speed = state.forwardSpeed;
    const bool wantsReverse =
            mControls.brake > 0.1f && mControls.throttle < 0.1f && fabsf(speed) < 0.3f;
    mReverseHold =
            wantsReverse ? mReverseHold + static_cast<float>(getPhysicsProcessDeltaTime()) : 0.0f;
    if (!mReverse && mReverseHold >= 0.5f)
    {
        mReverse = true;
        server->groundVehicleSelectGear(mVehicle, -1);
    }
    else if (mReverse && mControls.throttle > 0.1f && speed > -0.3f)
    {
        mReverse = false;
        server->groundVehicleSelectGear(mVehicle, 1);
    }
    PhysicsGroundVehicleControls controls;
    controls.throttle = mReverse ? mControls.brake : mControls.throttle;
    controls.brake = mReverse ? mControls.throttle : mControls.brake;
    controls.handbrake = mControls.handbrake;
    controls.steering = mControls.steering;
    server->groundVehicleSetControls(mVehicle, controls);
}

void GroundVehicle3D::placeWheels()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mVehicle.valid()) return;
    for (int i = 0; i < kWheels; ++i)
    {
        Node3D* wheel = wheelNode(i);
        PhysicsGroundWheelState state;
        if (!wheel || !server->groundVehicleGetWheelState(mVehicle, i, state)) continue;
        wheel->setVisible(!state.detached);
        wheel->setTransform(state.local);
    }
}

void GroundVehicle3D::bodyMoved(const Math::Mat4& frame)
{
    RigidBody3D::bodyMoved(frame);
    placeWheels();
}

void GroundVehicle3D::notificationHandler(int what)
{
    RigidBody3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPhysicsProcess:
        {
            PhysicsServer3D* server = PhysicsServer3D::singleton();
            if (!server) break;
            if (mDirty || (!server->groundVehicleValid(mVehicle) && isSimulated())) build();
            pushControls();
            updateSkids();
            break;
        }
        case kNotificationExitTree:
            release();
            mDirty = true;
            break;
        default:
            break;
    }
}

void GroundVehicle3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = GroundVehicle3D;
    DB::addPropertyGroup<P>("Vehicle", "");
    DB::addProperty<P>(PropertyInfo::enumeration("drive", "Front Wheel,Rear Wheel,All Wheel"),
            &P::setDrive, &P::getDrive);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "automatic"), &P::setAutomatic,
            &P::isAutomatic);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "anti_lock"), &P::setAntiLock,
            &P::isAntiLock);
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("wheel_ray_mask"), &P::setWheelRayMask,
            &P::getWheelRayMask);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "wheel_radius", 0.0, 2.0, 0.001),
            &P::setWheelRadius, &P::getWheelRadiusSetting);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "power", 0.01, 10.0, 0.01, true),
            &P::setPower, &P::getPower);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "grip", 0.01, 5.0, 0.01, true),
            &P::setGrip, &P::getGrip);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "roll_influence", 0.0, 1.0, 0.01),
            &P::setRollInfluence, &P::getRollInfluence);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "top_speed_kmh", 0.0, 1000.0, 1.0, true),
            &P::setTopSpeedKmh, &P::getTopSpeedKmh);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "handbrake_torque", 0.0, 20000.0, 10.0, true),
            &P::setHandbrakeTorque, &P::getHandbrakeTorque);

    DB::addPropertyGroup<P>("Controls", "");
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "throttle", 0.0, 1.0, 0.01),
            &P::setThrottle, &P::getThrottle);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "brake", 0.0, 1.0, 0.01),
            &P::setBrake, &P::getBrake);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "handbrake", 0.0, 1.0, 0.01),
            &P::setHandbrake, &P::getHandbrake);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "steering", -1.0, 1.0, 0.01),
            &P::setSteering, &P::getSteering);

    DB::addPropertyGroup<P>("Wheels", "wheel_");
    DB::addProperty<P>(PropertyInfo::nodePath("wheel_front_left", "Node3D"), &P::setWheelFrontLeft,
            &P::getWheelFrontLeft);
    DB::addProperty<P>(PropertyInfo::nodePath("wheel_front_right", "Node3D"),
            &P::setWheelFrontRight, &P::getWheelFrontRight);
    DB::addProperty<P>(PropertyInfo::nodePath("wheel_rear_left", "Node3D"), &P::setWheelRearLeft,
            &P::getWheelRearLeft);
    DB::addProperty<P>(PropertyInfo::nodePath("wheel_rear_right", "Node3D"), &P::setWheelRearRight,
            &P::getWheelRearRight);
    DB::addProperty<P>(PropertyInfo::nodePath("skid_marks", "SkidMarks3D"), &P::setSkidMarksPath,
            &P::getSkidMarksPath);
    DB::addProperty<P>(PropertyInfo::file("collision_model", "*.glb,*.gltf,*.pmesh,*.obj"),
            &P::setCollisionMesh, &P::getCollisionMesh);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "collision_scale"), &P::setCollisionScale,
            &P::getCollisionScale);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "collision_offset"), &P::setCollisionOffset,
            &P::getCollisionOffset);
}

} // namespace prisma3d
