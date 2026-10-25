#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/function.hpp>

namespace prisma3d
{

class BoneAttachment3D;
class SkeletonModifier3D;

class Skeleton3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Skeleton3D, Node3D);
    static void bindProperties();

    enum class ModifierCallbackModeProcess : std::uint8_t
    {
        Physics,
        Idle,
        Manual
    };

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    Skeleton3D();
    ~Skeleton3D() override;

    SkeletonId getSkeletonId() const;

    void setSkeleton(const Skeleton& skeleton);
    const Skeleton& getSkeleton() const;
    void clearBones();

    std::uint32_t getBoneCount() const;
    int findBone(const char* name) const;
    const ct::String& getBoneName(int bone) const;
    int getBoneParent(int bone) const;
    Math::Mat4 getBoneRest(int bone) const;
    const LocalPose& getBoneRestPose(int bone) const;
    Math::Mat4 getBoneGlobalRest(int bone) const;

    void setBonePose(int bone, const LocalPose& pose);
    const LocalPose& getBonePose(int bone) const;
    void setBonePosePosition(int bone, const Math::Vec3& position);
    void setBonePoseRotation(int bone, const Math::Quaternion& rotation);
    void setBonePoseScale(int bone, const Math::Vec3& scale);
    Math::Vec3 getBonePosePosition(int bone) const;
    Math::Quaternion getBonePoseRotation(int bone) const;
    Math::Vec3 getBonePoseScale(int bone) const;
    void resetBonePose(int bone);
    void resetBonePoses();

    Math::Mat4 getBoneGlobalPose(int bone);
    void setBoneGlobalPose(int bone, const Math::Mat4& pose);
    const ct::Vector<Math::Mat4>& getBonePalette();

    void forceUpdateAllBoneTransforms();
    void forceUpdateDeferred();
    void updateSkeleton();
    bool isDirty() const;
    std::uint64_t getPoseVersion() const;

    void setModifierCallbackModeProcess(ModifierCallbackModeProcess mode);
    ModifierCallbackModeProcess getModifierCallbackModeProcess() const;
    void advance(double delta);

    ct::Function<void()> poseUpdated;

protected:
    void notificationHandler(int what) override;
    void addChildNotify(Node* child) override;
    void removeChildNotify(Node* child) override;
    void moveChildNotify(Node* child) override;

private:
    friend class BoneAttachment3D;

    void makeDirty();
    void pushToServer();
    void bindAttachment(BoneAttachment3D* attachment);
    void unbindAttachment(BoneAttachment3D* attachment);
    void notifyAttachments();
    void processChanged();
    void makeModifiersDirty();
    void findModifiers();
    void processModifiers();
    void markBoneModified(int bone);

    Skeleton mSkeleton;
    ct::Vector<LocalPose> mRest;
    ct::Vector<LocalPose> mPose;
    ct::Vector<Math::Mat4> mGlobalPose;
    ct::Vector<Math::Mat4> mPalette;
    SkeletonId mServerSkeleton;
    std::uint64_t mPoseVersion;
    bool mDirty;
    bool mGlobalDirty;
    bool mUpdating;
    ct::Vector<BoneAttachment3D*> mAttachments;
    ModifierCallbackModeProcess mModifierCallbackModeProcess;
    ct::Vector<SkeletonModifier3D*> mModifiers;
    bool mModifiersDirty;
    bool mModifierUpdating;
    bool mDestroyed;
    double mUpdateDelta;
    ct::Vector<LocalPose> mPoseBackup;
    ct::Vector<LocalPose> mModifierBefore;
    ct::Vector<std::uint8_t> mModifierApplied;
};

} // namespace prisma3d
