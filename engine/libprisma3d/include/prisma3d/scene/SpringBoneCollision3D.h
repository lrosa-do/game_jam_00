#pragma once

#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>

namespace prisma3d
{

class Skeleton3D;

class SpringBoneCollision3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(SpringBoneCollision3D, Node3D);
    static void bindProperties();

    SpringBoneCollision3D();

    Skeleton3D* getSkeleton() const;

    void setBoneName(const ct::String& name);
    const ct::String& getBoneName() const;
    void setBone(int bone);
    int getBone() const;

    void setPositionOffset(const Math::Vec3& offset);
    Math::Vec3 getPositionOffset() const;
    void setRotationOffset(const Math::Quaternion& offset);
    Math::Quaternion getRotationOffset() const;

    void syncPose();
    Math::Mat4 getTransformFromSkeleton(const Math::Mat4& center) const;

    Math::Vec3 collide(const Math::Mat4& center, float boneRadius, float boneLength,
            const Math::Vec3& current) const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

    virtual Math::Vec3 collideImpl(const Math::Mat4& center, float boneRadius, float boneLength,
            const Math::Vec3& current) const;

private:
    void validateBoneName();

    ct::String mBoneName;
    int mBone;
    Math::Vec3 mPositionOffset;
    Math::Quaternion mRotationOffset;
};

class SpringBoneCollisionSphere3D : public SpringBoneCollision3D
{
public:
    PRISMA3D_NODE_CLASS(SpringBoneCollisionSphere3D, SpringBoneCollision3D);
    static void bindProperties();

    SpringBoneCollisionSphere3D();

    void setRadius(float radius);
    float getRadius() const;
    void setInside(bool enabled);
    bool isInside() const;

    static Math::Vec3 collideSphere(const Math::Vec3& origin, float radius, bool inside,
            float boneRadius, float boneLength, const Math::Vec3& current);

protected:
    Math::Vec3 collideImpl(const Math::Mat4& center, float boneRadius, float boneLength,
            const Math::Vec3& current) const override;

private:
    float mRadius;
    bool mInside;
};

class SpringBoneCollisionCapsule3D : public SpringBoneCollision3D
{
public:
    PRISMA3D_NODE_CLASS(SpringBoneCollisionCapsule3D, SpringBoneCollision3D);
    static void bindProperties();

    struct HeadAndTail
    {
        Math::Vec3 head;
        Math::Vec3 tail;
    };

    SpringBoneCollisionCapsule3D();

    void setRadius(float radius);
    float getRadius() const;
    void setHeight(float height);
    float getHeight() const;
    void setMidHeight(float midHeight);
    float getMidHeight() const;
    void setInside(bool enabled);
    bool isInside() const;

    HeadAndTail getHeadAndTail(const Math::Mat4& center) const;

protected:
    Math::Vec3 collideImpl(const Math::Mat4& center, float boneRadius, float boneLength,
            const Math::Vec3& current) const override;

private:
    float mRadius;
    float mHeight;
    bool mInside;
};

class SpringBoneCollisionPlane3D : public SpringBoneCollision3D
{
public:
    PRISMA3D_NODE_CLASS(SpringBoneCollisionPlane3D, SpringBoneCollision3D);

    SpringBoneCollisionPlane3D();

protected:
    Math::Vec3 collideImpl(const Math::Mat4& center, float boneRadius, float boneLength,
            const Math::Vec3& current) const override;
};

} // namespace prisma3d
