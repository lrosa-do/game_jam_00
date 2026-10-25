#include "prisma3d/scene/SpringBoneCollision3D.h"

#include "ModifierMath.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SpringBoneSimulator3D.h"

#include "platform.h"

namespace prisma3d
{

SpringBoneCollision3D::SpringBoneCollision3D()
        : Node3D(),
          mBoneName(),
          mBone(-1),
          mPositionOffset(0.0f, 0.0f, 0.0f),
          mRotationOffset()
{
}

Skeleton3D* SpringBoneCollision3D::getSkeleton() const
{
    Node* parent = getParent();
    SpringBoneSimulator3D* simulator = parent ? parent->castTo<SpringBoneSimulator3D>() : nullptr;
    if (!simulator) return nullptr;
    return simulator->getSkeleton();
}

void SpringBoneCollision3D::validateBoneName()
{
    if (!mBoneName.empty()) setBoneName(mBoneName);
    else if (mBone != -1)
        setBone(mBone);
}

void SpringBoneCollision3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name == "bone_name")
    {
        const Skeleton3D* skeleton = getSkeleton();
        if (skeleton)
        {
            info.hint = PropertyHint::Enum;
            info.hintString = ct::String();
            for (std::uint32_t i = 0; i < skeleton->getBoneCount(); ++i)
            {
                if (i > 0) info.hintString += ',';
                info.hintString += skeleton->getBoneName(static_cast<int>(i));
            }
        }
        else
        {
            info.hint = PropertyHint::None;
            info.hintString = ct::String();
        }
    }
    else if (mBone < 0 && (info.name == "position_offset" || info.name == "rotation_offset"))
    {
        info.usage = PropertyUsage::None;
    }
}

void SpringBoneCollision3D::setBoneName(const ct::String& name)
{
    mBoneName = name;
    Skeleton3D* skeleton = getSkeleton();
    if (skeleton) setBone(skeleton->findBone(mBoneName.c_str()));
}

const ct::String& SpringBoneCollision3D::getBoneName() const
{
    return mBoneName;
}

void SpringBoneCollision3D::setBone(int bone)
{
    mBone = bone;

    Skeleton3D* skeleton = getSkeleton();
    if (skeleton)
    {
        if (mBone <= -1 || static_cast<std::uint32_t>(mBone) >= skeleton->getBoneCount())
        {
            log_warn("spring bone collision: bone index '%d' is out of range", bone);
            mBone = -1;
        }
        else
        {
            mBoneName = skeleton->getBoneName(mBone);
        }
    }

    notifyPropertyListChanged();
}

int SpringBoneCollision3D::getBone() const
{
    return mBone;
}

void SpringBoneCollision3D::setPositionOffset(const Math::Vec3& offset)
{
    if (mPositionOffset.x == offset.x && mPositionOffset.y == offset.y &&
            mPositionOffset.z == offset.z)
        return;
    mPositionOffset = offset;
    syncPose();
}

Math::Vec3 SpringBoneCollision3D::getPositionOffset() const
{
    return mPositionOffset;
}

void SpringBoneCollision3D::setRotationOffset(const Math::Quaternion& offset)
{
    if (mRotationOffset.x == offset.x && mRotationOffset.y == offset.y &&
            mRotationOffset.z == offset.z && mRotationOffset.w == offset.w)
        return;
    mRotationOffset = offset;
    syncPose();
}

Math::Quaternion SpringBoneCollision3D::getRotationOffset() const
{
    return mRotationOffset;
}

void SpringBoneCollision3D::syncPose()
{
    if (mBone < 0) return;
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton) return;
    Math::Mat4 transform = skeleton->getGlobalTransform() * skeleton->getBoneGlobalPose(mBone);
    const Math::Vec3 origin = transform.GetTranslation() +
                              modifiermath::rotationQuaternion(transform) * mPositionOffset;
    const Math::Mat3 basis = transform.UpperLeft3x3() * mRotationOffset.ToMat3();
    setGlobalTransform(Math::Mat4(basis, origin));
}

Math::Mat4 SpringBoneCollision3D::getTransformFromSkeleton(const Math::Mat4& center) const
{
    Math::Mat4 transform = getGlobalTransform();
    Skeleton3D* skeleton = getSkeleton();
    if (skeleton)
    {
        const Math::Mat4 skeletonTransform = skeleton->getGlobalTransform();
        transform = skeletonTransform.InverseAffine() * center * transform;
    }
    return transform;
}

void SpringBoneCollision3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
        case kNotificationParented:
            validateBoneName();
            break;
        default:
            break;
    }
}

Math::Vec3 SpringBoneCollision3D::collide(const Math::Mat4& center, float boneRadius,
        float boneLength, const Math::Vec3& current) const
{
    return collideImpl(center, boneRadius, boneLength, current);
}

Math::Vec3 SpringBoneCollision3D::collideImpl(const Math::Mat4&, float, float,
        const Math::Vec3&) const
{
    return Math::Vec3(0.0f, 0.0f, 0.0f);
}

void SpringBoneCollision3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SpringBoneCollision3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "bone_name"), &P::setBoneName,
            &P::getBoneName);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "bone", PropertyHint::None, "",
                               PropertyUsage::NoEditor),
            &P::setBone, &P::getBone);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "position_offset"), &P::setPositionOffset,
            &P::getPositionOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Quat, "rotation_offset"), &P::setRotationOffset,
            &P::getRotationOffset);
}

SpringBoneCollisionSphere3D::SpringBoneCollisionSphere3D()
        : SpringBoneCollision3D(),
          mRadius(0.1f),
          mInside(false)
{
}

void SpringBoneCollisionSphere3D::setRadius(float radius)
{
    mRadius = radius;
}

float SpringBoneCollisionSphere3D::getRadius() const
{
    return mRadius;
}

void SpringBoneCollisionSphere3D::setInside(bool enabled)
{
    mInside = enabled;
}

bool SpringBoneCollisionSphere3D::isInside() const
{
    return mInside;
}

void SpringBoneCollisionSphere3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SpringBoneCollisionSphere3D;
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "radius", 0.0, 1.0, 0.001, true),
            &P::setRadius, &P::getRadius);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "inside"), &P::setInside, &P::isInside);
}

Math::Vec3 SpringBoneCollisionSphere3D::collideSphere(const Math::Vec3& origin, float radius,
        bool inside, float boneRadius, float, const Math::Vec3& current)
{
    const Math::Vec3 diff = current - origin;
    const float length = diff.Length();
    const float r = inside ? radius - boneRadius : boneRadius + radius;
    const float distance = inside ? r - length : length - r;
    if (distance > 0.0f) return current;
    return origin + modifiermath::normalized(diff) * r;
}

Math::Vec3 SpringBoneCollisionSphere3D::collideImpl(const Math::Mat4& center, float boneRadius,
        float boneLength, const Math::Vec3& current) const
{
    return collideSphere(getTransformFromSkeleton(center).GetTranslation(), mRadius, mInside,
            boneRadius, boneLength, current);
}

SpringBoneCollisionCapsule3D::SpringBoneCollisionCapsule3D()
        : SpringBoneCollision3D(),
          mRadius(0.1f),
          mHeight(0.5f),
          mInside(false)
{
}

void SpringBoneCollisionCapsule3D::setRadius(float radius)
{
    mRadius = radius;
    if (mRadius > mHeight * 0.5f) mHeight = mRadius * 2.0f;
}

float SpringBoneCollisionCapsule3D::getRadius() const
{
    return mRadius;
}

void SpringBoneCollisionCapsule3D::setHeight(float height)
{
    mHeight = height;
    if (mRadius > mHeight * 0.5f) mRadius = mHeight * 0.5f;
}

float SpringBoneCollisionCapsule3D::getHeight() const
{
    return mHeight;
}

void SpringBoneCollisionCapsule3D::setMidHeight(float midHeight)
{
    if (midHeight < 0.0f)
    {
        log_error("spring bone collision: capsule mid-height cannot be negative");
        return;
    }
    mHeight = midHeight + mRadius * 2.0f;
}

float SpringBoneCollisionCapsule3D::getMidHeight() const
{
    return mHeight - mRadius * 2.0f;
}

void SpringBoneCollisionCapsule3D::setInside(bool enabled)
{
    mInside = enabled;
}

bool SpringBoneCollisionCapsule3D::isInside() const
{
    return mInside;
}

SpringBoneCollisionCapsule3D::HeadAndTail SpringBoneCollisionCapsule3D::getHeadAndTail(
        const Math::Mat4& center) const
{
    const Math::Mat4 transform = getTransformFromSkeleton(center);
    const Math::Mat3 basis = transform.UpperLeft3x3();
    const Math::Vec3 origin = transform.GetTranslation();
    HeadAndTail result;
    result.head = origin + basis * (Math::Vec3(0.0f, 1.0f, 0.0f) * (mHeight * 0.5f - mRadius));
    result.tail = origin + basis * (Math::Vec3(0.0f, -1.0f, 0.0f) * (mHeight * 0.5f - mRadius));
    return result;
}

void SpringBoneCollisionCapsule3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SpringBoneCollisionCapsule3D;
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "radius", 0.0, 1.0, 0.001, true),
            &P::setRadius, &P::getRadius);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "height", 0.0, 1.0, 0.001, true),
            &P::setHeight, &P::getHeight);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "inside"), &P::setInside, &P::isInside);
}

Math::Vec3 SpringBoneCollisionCapsule3D::collideImpl(const Math::Mat4& center, float boneRadius,
        float boneLength, const Math::Vec3& current) const
{
    const HeadAndTail headTail = getHeadAndTail(center);
    const Math::Vec3 head = headTail.head;
    const Math::Vec3 tail = headTail.tail;
    const Math::Vec3 p = tail - head;
    const Math::Vec3 q = current - head;
    const float dot = p.Dot(q);
    if (dot <= 0.0f)
        return SpringBoneCollisionSphere3D::collideSphere(head, mRadius, mInside, boneRadius,
                boneLength, current);
    const float pls = p.LengthSquared();
    if (modifiermath::isZeroApprox(pls)) return current;
    if (pls <= dot)
        return SpringBoneCollisionSphere3D::collideSphere(head + p, mRadius, mInside, boneRadius,
                boneLength, current);
    return SpringBoneCollisionSphere3D::collideSphere(head + p * (dot / pls), mRadius, mInside,
            boneRadius, boneLength, current);
}

SpringBoneCollisionPlane3D::SpringBoneCollisionPlane3D()
        : SpringBoneCollision3D()
{
}

Math::Vec3 SpringBoneCollisionPlane3D::collideImpl(const Math::Mat4& center, float boneRadius,
        float, const Math::Vec3& current) const
{
    const Math::Mat4 transform = getTransformFromSkeleton(center);
    const Math::Vec3 position = transform.GetTranslation();
    const Math::Vec3 normal =
            modifiermath::rotationQuaternion(transform) * Math::Vec3(0.0f, 1.0f, 0.0f);
    const Math::Vec3 toVec = current - position;
    const float distance = toVec.Dot(normal) - boneRadius;
    if (distance > 0.0f) return current;
    return current + normal * -distance;
}

} // namespace prisma3d
