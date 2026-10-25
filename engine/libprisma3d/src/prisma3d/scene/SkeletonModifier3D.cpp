#include "prisma3d/scene/SkeletonModifier3D.h"

#include "ModifierMath.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Skeleton3D.h"

#include <cmath>

namespace prisma3d
{

namespace
{

constexpr double kAlmostOne = 1.0 - 0.00001;

} // namespace

const char* SkeletonModifier3D::boneAxisHint()
{
    return "+X,-X,+Y,-Y,+Z,-Z";
}

const char* SkeletonModifier3D::boneDirectionHint()
{
    return "+X,-X,+Y,-Y,+Z,-Z,FromParent";
}

const char* SkeletonModifier3D::secondaryDirectionHint()
{
    return "None,+X,-X,+Y,-Y,+Z,-Z,Custom";
}

const char* SkeletonModifier3D::rotationAxisHint()
{
    return "X,Y,Z,All,Custom";
}

SkeletonModifier3D::SkeletonModifier3D()
        : Node3D(),
          mActive(true),
          mInfluence(1.0f),
          mSkeleton(nullptr)
{
}

Skeleton3D* SkeletonModifier3D::getSkeleton() const
{
    return mSkeleton;
}

void SkeletonModifier3D::updateSkeletonPath()
{
    mSkeleton = nullptr;
    Node* parent = getParent();
    if (parent) mSkeleton = parent->castTo<Skeleton3D>();
}

void SkeletonModifier3D::updateSkeleton()
{
    if (!isInsideTree()) return;
    Skeleton3D* oldSkeleton = getSkeleton();
    updateSkeletonPath();
    Skeleton3D* newSkeleton = getSkeleton();
    if (oldSkeleton != newSkeleton) skeletonChanged(oldSkeleton, newSkeleton);
    if (newSkeleton) validateBoneNames();
}

void SkeletonModifier3D::skeletonChanged(Skeleton3D*, Skeleton3D*)
{
}

void SkeletonModifier3D::validateBoneNames()
{
}

void SkeletonModifier3D::forceUpdateSkeletonSkin()
{
    if (!isInsideTree()) return;
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton) return;
    skeleton->forceUpdateDeferred();
}

bool SkeletonModifier3D::hasProcess() const
{
    return false;
}

void SkeletonModifier3D::restUpdated()
{
}

void SkeletonModifier3D::setActive(bool active)
{
    if (mActive == active) return;
    mActive = active;
    activeChanged(mActive);
    forceUpdateSkeletonSkin();
}

bool SkeletonModifier3D::isActive() const
{
    return mActive;
}

void SkeletonModifier3D::activeChanged(bool)
{
}

void SkeletonModifier3D::setInfluence(float influence)
{
    mInfluence = influence;
}

float SkeletonModifier3D::getInfluence() const
{
    return mInfluence;
}

void SkeletonModifier3D::processModification(double delta)
{
    if (!isInsideTree() || !mActive) return;
    processModificationImpl(delta);
}

void SkeletonModifier3D::processModificationImpl(double)
{
}

void SkeletonModifier3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
        case kNotificationParented:
            updateSkeleton();
            break;
        case kNotificationExitTree:
            forceUpdateSkeletonSkin();
            break;
        case kNotificationUnparented:
            forceUpdateSkeletonSkin();
            mSkeleton = nullptr;
            break;
        default:
            break;
    }
}

void SkeletonModifier3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SkeletonModifier3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "active"), &P::setActive, &P::isActive);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "influence", 0.0, 1.0, 0.001),
            &P::setInfluence, &P::getInfluence);
}

Math::Vec3 SkeletonModifier3D::getVectorFromBoneAxis(BoneAxis axis)
{
    Math::Vec3 result;
    switch (axis)
    {
        case BoneAxis::PlusX:
            result = Math::Vec3(1.0f, 0.0f, 0.0f);
            break;
        case BoneAxis::MinusX:
            result = Math::Vec3(-1.0f, 0.0f, 0.0f);
            break;
        case BoneAxis::PlusY:
            result = Math::Vec3(0.0f, 1.0f, 0.0f);
            break;
        case BoneAxis::MinusY:
            result = Math::Vec3(0.0f, -1.0f, 0.0f);
            break;
        case BoneAxis::PlusZ:
            result = Math::Vec3(0.0f, 0.0f, 1.0f);
            break;
        case BoneAxis::MinusZ:
            result = Math::Vec3(0.0f, 0.0f, -1.0f);
            break;
    }
    return result;
}

Math::Vec3 SkeletonModifier3D::getVectorFromAxis(Axis axis)
{
    Math::Vec3 result;
    switch (axis)
    {
        case Axis::X:
            result = Math::Vec3(1.0f, 0.0f, 0.0f);
            break;
        case Axis::Y:
            result = Math::Vec3(0.0f, 1.0f, 0.0f);
            break;
        case Axis::Z:
            result = Math::Vec3(0.0f, 0.0f, 1.0f);
            break;
    }
    return result;
}

SkeletonModifier3D::Axis SkeletonModifier3D::getAxisFromBoneAxis(BoneAxis axis)
{
    Axis result = Axis::X;
    switch (axis)
    {
        case BoneAxis::PlusX:
        case BoneAxis::MinusX:
            result = Axis::X;
            break;
        case BoneAxis::PlusY:
        case BoneAxis::MinusY:
            result = Axis::Y;
            break;
        case BoneAxis::PlusZ:
        case BoneAxis::MinusZ:
            result = Axis::Z;
            break;
    }
    return result;
}

Math::Vec3 SkeletonModifier3D::limitLength(const Math::Vec3& origin, const Math::Vec3& destination,
        float length)
{
    return origin + modifiermath::normalized(destination - origin) * length;
}

Math::Quaternion SkeletonModifier3D::getLocalPoseRotation(Skeleton3D* skeleton, int bone,
        const Math::Quaternion& globalPoseRotation)
{
    const int parent = skeleton->getBoneParent(bone);
    if (parent < 0) return globalPoseRotation;
    return (modifiermath::rotationQuaternion(skeleton->getBoneGlobalPose(parent)).Inverse() *
            globalPoseRotation)
            .Normalized();
}

Math::Quaternion SkeletonModifier3D::getFromToRotation(const Math::Vec3& from, const Math::Vec3& to,
        const Math::Quaternion& prevRotation)
{
    if (modifiermath::isEqualApprox(from.Dot(to), -1.0f)) return prevRotation;
    const Math::Vec3 axis = from.Cross(to);
    if (modifiermath::isZeroApprox(axis)) return prevRotation;
    float angle = modifiermath::angleTo(from, to);
    if (modifiermath::isZeroApprox(angle)) angle = 0.0f;
    return modifiermath::quaternionFromAxisAngle(modifiermath::normalized(axis), angle);
}

Math::Quaternion SkeletonModifier3D::getFromToRotationByAxis(const Math::Vec3& from,
        const Math::Vec3& to, const Math::Vec3& axis)
{
    const double dot = from.Dot(to);
    if (dot > kAlmostOne) return Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    if (dot < -kAlmostOne)
        return modifiermath::quaternionFromAxisAngle(axis, static_cast<float>(modifiermath::kPi));
    double angle = modifiermath::angleTo(from, to);
    const Math::Vec3 cross = from.Cross(to);
    if (std::signbit(cross.Dot(axis))) angle = -angle;
    return modifiermath::quaternionFromAxisAngle(axis, static_cast<float>(angle));
}

Math::Quaternion SkeletonModifier3D::getSwing(const Math::Quaternion& rotation,
        const Math::Vec3& axis)
{
    if (modifiermath::isZeroApprox(axis)) return rotation;
    Math::Quaternion rot = rotation;
    if (!modifiermath::isNormalized(rot)) rot.Normalize();
    const Math::Vec3 unitAxis = modifiermath::normalized(axis);
    const Math::Vec3 v(rot.x, rot.y, rot.z);
    const float projectionLength = v.Dot(unitAxis);
    const Math::Vec3 twistVector = unitAxis * projectionLength;
    Math::Quaternion twist(twistVector.x, twistVector.y, twistVector.z, rot.w);
    if (!modifiermath::isNormalized(twist))
    {
        if (modifiermath::isZeroApprox(twist.LengthSquared())) return rot;
        twist.Normalize();
    }
    Math::Quaternion swing = rot * twist.Inverse();
    swing.Normalize();
    return swing;
}

Math::Vec3 SkeletonModifier3D::snapVectorToPlane(const Math::Vec3& planeNormal,
        const Math::Vec3& vector)
{
    if (modifiermath::isZeroApprox(planeNormal.LengthSquared())) return vector;
    const double length = vector.Length();
    const Math::Vec3 normalizedVector = modifiermath::normalized(vector);
    const Math::Vec3 normal = modifiermath::normalized(planeNormal);
    return modifiermath::slide(normalizedVector, normal) * static_cast<float>(length);
}

double SkeletonModifier3D::symmetrizeAngle(double angle)
{
    const double wrapped = modifiermath::fposmod(angle, modifiermath::kTau);
    return wrapped > modifiermath::kPi ? wrapped - modifiermath::kTau : wrapped;
}

double SkeletonModifier3D::getRollAngle(const Math::Quaternion& rotation,
        const Math::Vec3& rollAxis)
{
    const Math::Vec3 axis = modifiermath::normalized(rollAxis);

    const double dot = rotation.x * axis.x + rotation.y * axis.y + rotation.z * axis.z;

    Math::Quaternion rollComponent;
    rollComponent.x = static_cast<float>(axis.x * dot);
    rollComponent.y = static_cast<float>(axis.y * dot);
    rollComponent.z = static_cast<float>(axis.z * dot);
    rollComponent.w = rotation.w;

    const double length = rollComponent.Length();
    if (length > 0.00001)
    {
        rollComponent = Math::Quaternion(static_cast<float>(rollComponent.x / length),
                static_cast<float>(rollComponent.y / length),
                static_cast<float>(rollComponent.z / length),
                static_cast<float>(rollComponent.w / length));
    }
    else
    {
        return 0.0;
    }

    const double clamped =
            rollComponent.w < -1.0f ? -1.0 : (rollComponent.w > 1.0f ? 1.0 : rollComponent.w);
    const double angle = 2.0 * std::acos(clamped);

    const double direction =
            (rollComponent.x * axis.x + rollComponent.y * axis.y + rollComponent.z * axis.z > 0.0f)
                    ? 1.0
                    : -1.0;

    return symmetrizeAngle(angle * direction);
}

Math::Vec3 SkeletonModifier3D::getProjectedNormal(const Math::Vec3& a, const Math::Vec3& b,
        const Math::Vec3& point)
{
    const Math::Vec3 direction = b - a;
    const float denominator = direction.LengthSquared();
    if (modifiermath::isZeroApprox(denominator)) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const Math::Vec3 w = point - a;
    const float t = w.Dot(direction) / denominator;
    const Math::Vec3 h = a + direction * t;
    return modifiermath::normalized(point - h);
}

} // namespace prisma3d
