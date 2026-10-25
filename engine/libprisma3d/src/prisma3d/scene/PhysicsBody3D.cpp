#include "prisma3d/scene/PhysicsBody3D.h"

#include "prisma3d/main/Engine.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ScriptInstance.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kRigidTolerance = 1e-4f;

bool nearlyEqualColumn(const Math::Vec4& a, const Math::Vec4& b, float tolerance)
{
    return fabsf(a.x - b.x) <= tolerance && fabsf(a.y - b.y) <= tolerance &&
           fabsf(a.z - b.z) <= tolerance && fabsf(a.w - b.w) <= tolerance;
}

bool nearlyEqual(const Math::Mat4& a, const Math::Mat4& b, float tolerance)
{
    return nearlyEqualColumn(a.col0, b.col0, tolerance) &&
           nearlyEqualColumn(a.col1, b.col1, tolerance) &&
           nearlyEqualColumn(a.col2, b.col2, tolerance) &&
           nearlyEqualColumn(a.col3, b.col3, tolerance);
}

float columnLength(const Math::Vec4& column)
{
    return sqrtf(column.x * column.x + column.y * column.y + column.z * column.z);
}

} // namespace

void PhysicsBody3D::Listener::physicsBodyMoved(const Math::Mat4& frame)
{
    if (owner) owner->bodyMoved(frame);
}

void PhysicsBody3D::Listener::physicsSleepingChanged(bool sleeping)
{
    if (owner && owner->sleepingChanged) owner->sleepingChanged(sleeping);
}

void PhysicsBody3D::Listener::physicsContactStarted(const PhysicsContact& contact)
{
    if (!owner) return;
    PhysicsBody3D* other = PhysicsBody3D::fromBody(contact.other);
    owner->contactStarted(other, contact);
    if (owner && owner->bodyEntered) owner->bodyEntered(other, contact);
    if (owner)
        if (ScriptInstance* script = owner->getScriptInstance()) script->bodyEntered(other);
}

void PhysicsBody3D::Listener::physicsContactEnded(const PhysicsContact& contact)
{
    if (!owner) return;
    PhysicsBody3D* other = PhysicsBody3D::fromBody(contact.other);
    if (owner->bodyExited) owner->bodyExited(other, contact);
    if (owner)
        if (ScriptInstance* script = owner->getScriptInstance()) script->bodyExited(other);
}

PhysicsBody3D::PhysicsBody3D(PhysicsBodyMode mode)
        : Node3D(),
          mListener(),
          mBody(),
          mMode(mode),
          mSynced(Math::Mat4::Identity()),
          mHasSynced(false),
          mContactMonitor(false),
          mMaxContacts(8),
          mLayer(1),
          mMask(1),
          mFriction(1.0f),
          mBounce(0.0f)
{
    defineAncestry(NodeClass::PhysicsBody3D);
    setNotifyTransform(true);
    mListener.owner = this;
    createBody();
}

bool PhysicsBody3D::createBody()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return false;
    mBody = server->bodyCreate();
    server->bodySetMode(mBody, mMode);
    server->bodySetListener(mBody, &mListener);
    server->bodySetUserData(mBody, this);
    server->bodySetInWorld(mBody, false);
    return true;
}

bool PhysicsBody3D::ensureBody()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return false;
    if (server->bodyValid(mBody)) return true;
    mHasSynced = false;
    if (!createBody()) return false;
    pushState();
    if (isInsideTree()) enterWorld();
    return true;
}

void PhysicsBody3D::pushState()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return;
    server->bodySetCollisionLayer(mBody, mLayer);
    server->bodySetCollisionMask(mBody, mMask);
    server->bodySetFriction(mBody, mFriction);
    server->bodySetBounce(mBody, mBounce);
    refreshContactMonitor();
}

bool PhysicsBody3D::needsContactMonitor() const
{
    return false;
}

void PhysicsBody3D::refreshContactMonitor()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->bodyValid(mBody)) return;
    const ScriptInstance* script = getScriptInstance();
    const bool needed = needsContactMonitor() || (script && script->wantsBodyContacts());
    const int maxContacts = needed && mMaxContacts <= 0 ? 8 : mMaxContacts;
    server->bodySetContactMonitor(mBody, mContactMonitor || needed, maxContacts);
}

void PhysicsBody3D::enterWorld()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->bodyValid(mBody)) return;
    mHasSynced = false;
    pushTransform();
    bindShape(nullptr);
    if (isEnabled()) server->bodySetInWorld(mBody, true);
}

PhysicsBody3D::~PhysicsBody3D()
{
    releaseJoints(true);
    mListener.owner = nullptr;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mBody.valid()) server->bodyFree(mBody);
}

PhysicsBodyId PhysicsBody3D::getBody() const
{
    return mBody;
}

PhysicsBodyMode PhysicsBody3D::getMode() const
{
    return mMode;
}

bool PhysicsBody3D::isSimulated() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server && server->bodyIsSimulated(mBody);
}

void PhysicsBody3D::setCollisionLayer(std::uint32_t layer)
{
    mLayer = layer;
    if (!ensureBody()) return;
    PhysicsServer3D::singleton()->bodySetCollisionLayer(mBody, layer);
}

std::uint32_t PhysicsBody3D::getCollisionLayer() const
{
    return mLayer;
}

void PhysicsBody3D::setCollisionMask(std::uint32_t mask)
{
    mMask = mask;
    if (!ensureBody()) return;
    PhysicsServer3D::singleton()->bodySetCollisionMask(mBody, mask);
}

std::uint32_t PhysicsBody3D::getCollisionMask() const
{
    return mMask;
}

void PhysicsBody3D::setFriction(float friction)
{
    mFriction = fmaxf(friction, 0.0f);
    if (!ensureBody()) return;
    PhysicsServer3D::singleton()->bodySetFriction(mBody, mFriction);
}

float PhysicsBody3D::getFriction() const
{
    return mFriction;
}

void PhysicsBody3D::setBounce(float bounce)
{
    mBounce = fminf(fmaxf(bounce, 0.0f), 1.0f);
    if (!ensureBody()) return;
    PhysicsServer3D::singleton()->bodySetBounce(mBody, mBounce);
}

float PhysicsBody3D::getBounce() const
{
    return mBounce;
}

void PhysicsBody3D::setContactMonitor(bool enabled, int maxContacts)
{
    const bool changed = mContactMonitor != enabled;
    mContactMonitor = enabled;
    mMaxContacts = maxContacts < 0 ? 0 : maxContacts;
    if (ensureBody()) refreshContactMonitor();
    if (changed) notifyPropertyListChanged();
}

void PhysicsBody3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (!mContactMonitor && info.name == "max_contacts_reported")
        info.usage = PropertyUsage::NoEditor;
}

bool PhysicsBody3D::isContactMonitor() const
{
    return mContactMonitor;
}

void PhysicsBody3D::setMaxContacts(int maxContacts)
{
    mMaxContacts = maxContacts < 0 ? 0 : maxContacts;
    if (ensureBody()) refreshContactMonitor();
}

int PhysicsBody3D::getMaxContacts() const
{
    return mMaxContacts;
}

PhysicsBody3D* PhysicsBody3D::fromBody(PhysicsBodyId body)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->bodyValid(body)) return nullptr;
    return static_cast<PhysicsBody3D*>(server->bodyGetUserData(body));
}

void PhysicsBody3D::contactStarted(PhysicsBody3D* other, const PhysicsContact& contact)
{
    (void) other;
    (void) contact;
}

void PhysicsBody3D::bodyMoved(const Math::Mat4& frame)
{
    if (mMode == PhysicsBodyMode::Rigid) writeTransform(frame);
}

void PhysicsBody3D::writeTransform(const Math::Mat4& frame)
{
    const Math::Mat4 current = getGlobalTransform();
    if (mHasSynced && !nearlyEqual(current, mSynced, kRigidTolerance))
    {
        pushTransform();
        return;
    }
    Math::Mat4 written = frame;
    const float sx = columnLength(current.col0);
    const float sy = columnLength(current.col1);
    const float sz = columnLength(current.col2);
    written.col0 = Math::Vec4(frame.col0.x * sx, frame.col0.y * sx, frame.col0.z * sx, 0.0f);
    written.col1 = Math::Vec4(frame.col1.x * sy, frame.col1.y * sy, frame.col1.z * sy, 0.0f);
    written.col2 = Math::Vec4(frame.col2.x * sz, frame.col2.y * sz, frame.col2.z * sz, 0.0f);
    mSynced = written;
    mHasSynced = true;
    setGlobalTransform(written);
}

void PhysicsBody3D::pushTransform()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mBody.valid() || !isInsideTree()) return;
    const Math::Mat4 frame = getGlobalTransform();
    const float tolerance = mMode == PhysicsBodyMode::Rigid ? kRigidTolerance : 0.0f;
    if (mHasSynced && nearlyEqual(frame, mSynced, tolerance)) return;
    mSynced = frame;
    mHasSynced = true;
    server->bodySetTransform(mBody, frame);
}

void PhysicsBody3D::syncToServer()
{
    pushTransform();
}

void PhysicsBody3D::attachJoint(Joint3D* joint)
{
    for (std::size_t i = 0; i < mJoints.size(); ++i)
        if (mJoints[i] == joint) return;
    mJoints.push_back(joint);
}

void PhysicsBody3D::detachJoint(Joint3D* joint)
{
    for (std::size_t i = 0; i < mJoints.size(); ++i)
    {
        if (mJoints[i] != joint) continue;
        mJoints.erase(mJoints.begin() + static_cast<std::ptrdiff_t>(i));
        return;
    }
}

void PhysicsBody3D::releaseJoints(bool forget)
{
    ct::Vector<Joint3D*> joints = mJoints;
    if (forget) mJoints.clear();
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        if (forget) joints[i]->bodyGone(this);
        else
            joints[i]->bodyLeftTree();
    }
}

void PhysicsBody3D::bindShape(const CollisionShape3D* ignore)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mBody.valid()) return;
    const CollisionShape3D* chosen = nullptr;
    int found = 0;
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        CollisionShape3D* shape = getChild(i)->castTo<CollisionShape3D>();
        if (!shape || shape == ignore || shape->isDisabled()) continue;
        shape->ensureShape();
        if (!server->shapeValid(shape->getShape())) continue;
        if (!chosen) chosen = shape;
        ++found;
    }
    if (found > 1)
        log_warn("physics: body '%s' has %d collision shapes, only the first is used",
                getName().c_str(), found);
    if (!chosen)
    {
        server->bodySetShape(mBody, PhysicsShapeId(), Math::Mat4::Identity());
        return;
    }
    server->bodySetShape(mBody, chosen->getShape(), chosen->getTransform());
}

void PhysicsBody3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return;
    if (what == kNotificationEnterTree && !server->bodyValid(mBody))
    {
        ensureBody();
        return;
    }
    if (!mBody.valid()) return;
    switch (what)
    {
        case kNotificationEnterTree:
            enterWorld();
            break;
        case kNotificationExitTree:
            releaseJoints(false);
            server->bodySetInWorld(mBody, false);
            break;
        case kNotificationTransformChanged:
            pushTransform();
            break;
        case kNotificationDisabled:
            if (isInsideTree()) server->bodySetInWorld(mBody, false);
            break;
        case kNotificationEnabled:
            if (isInsideTree()) server->bodySetInWorld(mBody, true);
            break;
        default:
            break;
    }
}

StaticBody3D::StaticBody3D()
        : PhysicsBody3D(PhysicsBodyMode::Static)
{
}

AnimatableBody3D::AnimatableBody3D()
        : PhysicsBody3D(PhysicsBodyMode::Kinematic)
{
}

RigidBody3D::RigidBody3D()
        : PhysicsBody3D(PhysicsBodyMode::Rigid),
          mMass(1.0f),
          mInertia(0.0f, 0.0f, 0.0f),
          mLockRotation(false),
          mLinearDamp(0.0f),
          mAngularDamp(0.0f),
          mLinearDampMode(PhysicsDampMode::Combine),
          mAngularDampMode(PhysicsDampMode::Combine),
          mCanSleep(true),
          mSleeping(false),
          mLinearVelocity(0.0f, 0.0f, 0.0f),
          mAngularVelocity(0.0f, 0.0f, 0.0f),
          mConstantForce(0.0f, 0.0f, 0.0f),
          mConstantTorque(0.0f, 0.0f, 0.0f),
          mInterpolated(true)
{
    setProcess(true);
}

#define PRISMA3D_BODY_CALL(call)                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (ensureBody()) PhysicsServer3D::singleton()->call;                                      \
    } while (0)

void RigidBody3D::pushState()
{
    PhysicsBody3D::pushState();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return;
    const PhysicsBodyId body = getBody();
    server->bodySetMass(body, mMass);
    server->bodySetInertia(body, mInertia);
    server->bodySetLockRotation(body, mLockRotation);
    server->bodySetLinearDamp(body, mLinearDamp, mLinearDampMode);
    server->bodySetAngularDamp(body, mAngularDamp, mAngularDampMode);
    server->bodySetCanSleep(body, mCanSleep);
    server->bodySetInterpolated(body, mInterpolated);
    server->bodySetConstantForce(body, mConstantForce);
    server->bodySetConstantTorque(body, mConstantTorque);
    server->bodySetLinearVelocity(body, mLinearVelocity);
    server->bodySetAngularVelocity(body, mAngularVelocity);
    server->bodySetSleeping(body, mSleeping);
}

void RigidBody3D::setMass(float mass)
{
    mMass = fmaxf(mass, 0.001f);
    PRISMA3D_BODY_CALL(bodySetMass(getBody(), mMass));
}

float RigidBody3D::getMass() const
{
    return mMass;
}

void RigidBody3D::setInertia(const Math::Vec3& inertia)
{
    mInertia = inertia;
    PRISMA3D_BODY_CALL(bodySetInertia(getBody(), mInertia));
}

Math::Vec3 RigidBody3D::getInertia() const
{
    return mInertia;
}

Math::Vec3 RigidBody3D::getComputedInertia() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->bodyGetInertia(getBody()) : mInertia;
}

void RigidBody3D::setLockRotation(bool lock)
{
    mLockRotation = lock;
    PRISMA3D_BODY_CALL(bodySetLockRotation(getBody(), lock));
}

bool RigidBody3D::isLockRotation() const
{
    return mLockRotation;
}

void RigidBody3D::setLinearDamp(float damp, PhysicsDampMode mode)
{
    mLinearDamp = fmaxf(damp, 0.0f);
    mLinearDampMode = mode;
    PRISMA3D_BODY_CALL(bodySetLinearDamp(getBody(), mLinearDamp, mLinearDampMode));
}

float RigidBody3D::getLinearDamp() const
{
    return mLinearDamp;
}

void RigidBody3D::setLinearDampMode(PhysicsDampMode mode)
{
    setLinearDamp(mLinearDamp, mode);
}

PhysicsDampMode RigidBody3D::getLinearDampMode() const
{
    return mLinearDampMode;
}

void RigidBody3D::setAngularDamp(float damp, PhysicsDampMode mode)
{
    mAngularDamp = fmaxf(damp, 0.0f);
    mAngularDampMode = mode;
    PRISMA3D_BODY_CALL(bodySetAngularDamp(getBody(), mAngularDamp, mAngularDampMode));
}

float RigidBody3D::getAngularDamp() const
{
    return mAngularDamp;
}

void RigidBody3D::setAngularDampMode(PhysicsDampMode mode)
{
    setAngularDamp(mAngularDamp, mode);
}

PhysicsDampMode RigidBody3D::getAngularDampMode() const
{
    return mAngularDampMode;
}

void RigidBody3D::setCanSleep(bool canSleep)
{
    mCanSleep = canSleep;
    PRISMA3D_BODY_CALL(bodySetCanSleep(getBody(), canSleep));
}

bool RigidBody3D::canSleep() const
{
    return mCanSleep;
}

void RigidBody3D::setSleeping(bool sleeping)
{
    mSleeping = sleeping;
    if (sleeping)
    {
        mLinearVelocity = Math::Vec3(0.0f, 0.0f, 0.0f);
        mAngularVelocity = Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    PRISMA3D_BODY_CALL(bodySetSleeping(getBody(), sleeping));
}

bool RigidBody3D::isSleeping() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && server->bodyValid(getBody())) return server->bodyIsSleeping(getBody());
    return mSleeping;
}

void RigidBody3D::setLinearVelocity(const Math::Vec3& velocity)
{
    mLinearVelocity = velocity;
    if (velocity.LengthSquared() > 0.0f) mSleeping = false;
    PRISMA3D_BODY_CALL(bodySetLinearVelocity(getBody(), velocity));
}

Math::Vec3 RigidBody3D::getLinearVelocity() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && server->bodyValid(getBody())) return server->bodyGetLinearVelocity(getBody());
    return mLinearVelocity;
}

void RigidBody3D::setAngularVelocity(const Math::Vec3& velocity)
{
    mAngularVelocity = velocity;
    if (velocity.LengthSquared() > 0.0f) mSleeping = false;
    PRISMA3D_BODY_CALL(bodySetAngularVelocity(getBody(), velocity));
}

Math::Vec3 RigidBody3D::getAngularVelocity() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && server->bodyValid(getBody())) return server->bodyGetAngularVelocity(getBody());
    return mAngularVelocity;
}

Math::Vec3 RigidBody3D::getVelocityAtPosition(const Math::Vec3& globalPosition) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->bodyGetVelocityAtPosition(getBody(), globalPosition)
                  : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBody3D::applyCentralImpulse(const Math::Vec3& impulse)
{
    PRISMA3D_BODY_CALL(bodyApplyCentralImpulse(getBody(), impulse));
}

void RigidBody3D::applyImpulse(const Math::Vec3& impulse, const Math::Vec3& offset)
{
    PRISMA3D_BODY_CALL(bodyApplyImpulse(getBody(), impulse, offset));
}

void RigidBody3D::applyTorqueImpulse(const Math::Vec3& impulse)
{
    PRISMA3D_BODY_CALL(bodyApplyTorqueImpulse(getBody(), impulse));
}

void RigidBody3D::applyCentralForce(const Math::Vec3& force)
{
    PRISMA3D_BODY_CALL(bodyApplyCentralForce(getBody(), force));
}

void RigidBody3D::applyForce(const Math::Vec3& force, const Math::Vec3& offset)
{
    PRISMA3D_BODY_CALL(bodyApplyForce(getBody(), force, offset));
}

void RigidBody3D::applyTorque(const Math::Vec3& torque)
{
    PRISMA3D_BODY_CALL(bodyApplyTorque(getBody(), torque));
}

void RigidBody3D::setConstantForce(const Math::Vec3& force)
{
    mConstantForce = force;
    PRISMA3D_BODY_CALL(bodySetConstantForce(getBody(), force));
}

Math::Vec3 RigidBody3D::getConstantForce() const
{
    return mConstantForce;
}

void RigidBody3D::setConstantTorque(const Math::Vec3& torque)
{
    mConstantTorque = torque;
    PRISMA3D_BODY_CALL(bodySetConstantTorque(getBody(), torque));
}

Math::Vec3 RigidBody3D::getConstantTorque() const
{
    return mConstantTorque;
}

#undef PRISMA3D_BODY_CALL

void RigidBody3D::setInterpolated(bool interpolated)
{
    mInterpolated = interpolated;
    if (ensureBody()) PhysicsServer3D::singleton()->bodySetInterpolated(getBody(), interpolated);
}

bool RigidBody3D::isInterpolated() const
{
    return mInterpolated;
}

int RigidBody3D::getContactCount() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->bodyGetContactCount(getBody()) : 0;
}

void RigidBody3D::getCollidingBodies(ct::Vector<PhysicsBody3D*>& out) const
{
    out.clear();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return;
    ct::Vector<PhysicsBodyId> ids;
    server->bodyGetCollidingBodies(getBody(), ids);
    for (std::size_t i = 0; i < ids.size(); ++i)
        if (PhysicsBody3D* other = PhysicsBody3D::fromBody(ids[i])) out.push_back(other);
}

void RigidBody3D::notificationHandler(int what)
{
    PhysicsBody3D::notificationHandler(what);
    if (what != kNotificationProcess || !mInterpolated || !isInsideTree()) return;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    Engine* engine = Engine::singleton();
    if (!server || !engine || !server->bodyIsSimulated(getBody()) ||
            server->bodyIsSleeping(getBody()))
        return;
    writeTransform(server->bodyGetInterpolatedTransform(getBody(),
            static_cast<float>(engine->getPhysicsInterpolationFraction())));
}

void PhysicsBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = PhysicsBody3D;
    DB::addPropertyGroup<P>("Collision", "collision_");
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("collision_layer"), &P::setCollisionLayer,
            &P::getCollisionLayer);
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("collision_mask"), &P::setCollisionMask,
            &P::getCollisionMask);

    DB::addPropertyGroup<P>("Material", "");
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "friction", 0.0, 1.0, 0.01, true),
            &P::setFriction, &P::getFriction);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "bounce", 0.0, 1.0, 0.01, true),
            &P::setBounce, &P::getBounce);

    DB::addPropertyGroup<P>("Contacts", "");
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Bool, "contact_monitor"),
            [](P* body, const PropertyValue& value)
            { body->setContactMonitor(value.asBool(), body->getMaxContacts()); },
            &P::isContactMonitor);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "max_contacts_reported", 0.0, 64.0, 1.0, true),
            &P::setMaxContacts, &P::getMaxContacts);
}

void StaticBody3D::bindProperties()
{
}

void AnimatableBody3D::bindProperties()
{
}

void RigidBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = RigidBody3D;
    const char* dampModes = "Combine,Replace";
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "mass", 0.001, 1000.0, 0.001, true),
            &P::setMass, &P::getMass);

    DB::addPropertyGroup<P>("Mass Distribution", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "inertia"), &P::setInertia, &P::getInertia);

    DB::addPropertyGroup<P>("Deactivation", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "sleeping"), &P::setSleeping,
            &P::isSleeping);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "can_sleep"), &P::setCanSleep,
            &P::canSleep);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "lock_rotation"), &P::setLockRotation,
            &P::isLockRotation);

    DB::addPropertyGroup<P>("Solver", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "interpolated"), &P::setInterpolated,
            &P::isInterpolated);

    DB::addPropertyGroup<P>("Linear", "linear_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "linear_velocity"), &P::setLinearVelocity,
            &P::getLinearVelocity);
    DB::addProperty<P>(PropertyInfo::enumeration("linear_damp_mode", dampModes),
            &P::setLinearDampMode, &P::getLinearDampMode);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "linear_damp", 0.0, 100.0, 0.001, true),
            [](P* body, const PropertyValue& value) {
                body->setLinearDamp(static_cast<float>(value.asFloat()), body->getLinearDampMode());
            },
            &P::getLinearDamp);

    DB::addPropertyGroup<P>("Angular", "angular_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "angular_velocity"), &P::setAngularVelocity,
            &P::getAngularVelocity);
    DB::addProperty<P>(PropertyInfo::enumeration("angular_damp_mode", dampModes),
            &P::setAngularDampMode, &P::getAngularDampMode);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "angular_damp", 0.0, 100.0, 0.001, true),
            [](P* body, const PropertyValue& value) {
                body->setAngularDamp(static_cast<float>(value.asFloat()),
                        body->getAngularDampMode());
            },
            &P::getAngularDamp);

    DB::addPropertyGroup<P>("Constant Forces", "constant_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "constant_force"), &P::setConstantForce,
            &P::getConstantForce);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "constant_torque"), &P::setConstantTorque,
            &P::getConstantTorque);
}

} // namespace prisma3d
