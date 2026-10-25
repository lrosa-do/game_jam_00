#pragma once

#include "prisma3d/scene/Node3D.h"

#include <cstdint>

namespace prisma3d
{

class Skeleton3D;

class SkeletonModifier3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(SkeletonModifier3D, Node3D);
    static void bindProperties();

    enum class BoneAxis : std::uint8_t
    {
        PlusX,
        MinusX,
        PlusY,
        MinusY,
        PlusZ,
        MinusZ
    };

    enum class BoneDirection : std::uint8_t
    {
        PlusX,
        MinusX,
        PlusY,
        MinusY,
        PlusZ,
        MinusZ,
        FromParent
    };

    enum class SecondaryDirection : std::uint8_t
    {
        None,
        PlusX,
        MinusX,
        PlusY,
        MinusY,
        PlusZ,
        MinusZ,
        Custom
    };

    enum class RotationAxis : std::uint8_t
    {
        X,
        Y,
        Z,
        All,
        Custom
    };

    enum class Axis : std::uint8_t
    {
        X,
        Y,
        Z
    };

    static const char* boneAxisHint();
    static const char* boneDirectionHint();
    static const char* secondaryDirectionHint();
    static const char* rotationAxisHint();

    SkeletonModifier3D();

    Skeleton3D* getSkeleton() const;

    void setActive(bool active);
    bool isActive() const;

    void setInfluence(float influence);
    float getInfluence() const;

    virtual bool hasProcess() const;
    virtual void restUpdated();

    void processModification(double delta);

    static Math::Vec3 getVectorFromBoneAxis(BoneAxis axis);
    static Math::Vec3 getVectorFromAxis(Axis axis);
    static Axis getAxisFromBoneAxis(BoneAxis axis);

    static Math::Vec3 limitLength(const Math::Vec3& origin, const Math::Vec3& destination,
            float length);
    static Math::Quaternion getLocalPoseRotation(Skeleton3D* skeleton, int bone,
            const Math::Quaternion& globalPoseRotation);
    static Math::Quaternion getFromToRotation(const Math::Vec3& from, const Math::Vec3& to,
            const Math::Quaternion& prevRotation);
    static Math::Quaternion getFromToRotationByAxis(const Math::Vec3& from, const Math::Vec3& to,
            const Math::Vec3& axis);
    static Math::Quaternion getSwing(const Math::Quaternion& rotation, const Math::Vec3& axis);
    static Math::Vec3 snapVectorToPlane(const Math::Vec3& planeNormal, const Math::Vec3& vector);
    static double symmetrizeAngle(double angle);
    static double getRollAngle(const Math::Quaternion& rotation, const Math::Vec3& rollAxis);
    static Math::Vec3 getProjectedNormal(const Math::Vec3& a, const Math::Vec3& b,
            const Math::Vec3& point);

protected:
    void notificationHandler(int what) override;

    virtual void skeletonChanged(Skeleton3D* oldSkeleton, Skeleton3D* newSkeleton);
    virtual void validateBoneNames();
    virtual void activeChanged(bool active);
    virtual void processModificationImpl(double delta);

    void forceUpdateSkeletonSkin();

private:
    void updateSkeletonPath();
    void updateSkeleton();

    bool mActive;
    float mInfluence;
    Skeleton3D* mSkeleton;
};

} // namespace prisma3d
