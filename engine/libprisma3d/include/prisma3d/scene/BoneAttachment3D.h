#pragma once

#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class Skeleton3D;

class BoneAttachment3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(BoneAttachment3D, Node3D);
    static void bindProperties();

    BoneAttachment3D();
    ~BoneAttachment3D() override;

    Skeleton3D* getSkeleton();

    void setBoneName(const ct::String& name);
    const ct::String& getBoneName() const;
    void setBoneIdx(int bone);
    int getBoneIdx() const;

    void setOverridePose(bool overridePose);
    bool getOverridePose() const;

    void setUseExternalSkeleton(bool useExternalSkeleton);
    bool getUseExternalSkeleton() const;
    void setExternalSkeleton(const ct::String& path);
    const ct::String& getExternalSkeleton() const;

    void onSkeletonUpdate();

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    friend class Skeleton3D;

    Skeleton3D* resolveExternalSkeleton();
    void checkBind();
    void checkUnbind();
    void transformChanged();

    Skeleton3D* mBoundSkeleton;
    ct::String mBoneName;
    int mBoneIdx;
    bool mOverridePose;
    bool mOverrideDirty;
    bool mOverriding;
    bool mUseExternalSkeleton;
    ct::String mExternalSkeleton;
    bool mUpdating;
};

} // namespace prisma3d
