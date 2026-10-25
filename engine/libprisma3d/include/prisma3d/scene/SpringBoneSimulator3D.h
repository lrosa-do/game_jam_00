#pragma once

#include "prisma3d/scene/SkeletonModifier3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class SpringBoneCollision3D;

class SpringBoneSimulator3D : public SkeletonModifier3D
{
public:
    PRISMA3D_NODE_CLASS(SpringBoneSimulator3D, SkeletonModifier3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    enum class CenterFrom : std::uint8_t
    {
        WorldOrigin,
        Node,
        Bone
    };

    struct VerletInfo
    {
        Math::Vec3 prevTail;
        Math::Vec3 currentTail;
        Math::Quaternion currentRot;
        Math::Vec3 forwardVector;
        float length = 0.0f;
    };

    struct DampingCurve
    {
        ct::Vector<float> samples;

        bool valid() const
        {
            return samples.size() > 0;
        }
        float sampleBaked(float offset) const;
    };

    struct JointSetting
    {
        ct::String boneName;
        int bone = -1;

        RotationAxis rotationAxis = RotationAxis::All;
        Math::Vec3 rotationAxisVector = Math::Vec3(1.0f, 0.0f, 0.0f);
        Math::Vec3 getRotationAxisVector() const;

        float radius = 0.1f;
        float stiffness = 1.0f;
        float drag = 0.0f;
        float gravity = 0.0f;
        Math::Vec3 gravityDirection = Math::Vec3(0.0f, -1.0f, 0.0f);

        bool hasVerlet = false;
        VerletInfo verlet;
    };

    struct Setting
    {
        bool jointsDirty = false;

        ct::String rootBoneName;
        int rootBone = -1;

        ct::String endBoneName;
        int endBone = -1;

        bool extendEndBone = false;
        BoneDirection endBoneDirection = BoneDirection::FromParent;
        float endBoneLength = 0.0f;

        CenterFrom centerFrom = CenterFrom::WorldOrigin;
        ct::String centerNode;
        ct::String centerBoneName;
        int centerBone = -1;

        bool individualConfig = false;
        float radius = 0.02f;
        DampingCurve radiusDampingCurve;
        float stiffness = 1.0f;
        DampingCurve stiffnessDampingCurve;
        float drag = 0.4f;
        DampingCurve dragDampingCurve;
        float gravity = 0.0f;
        DampingCurve gravityDampingCurve;
        Math::Vec3 gravityDirection = Math::Vec3(0.0f, -1.0f, 0.0f);
        RotationAxis rotationAxis = RotationAxis::All;
        Math::Vec3 rotationAxisVector = Math::Vec3(1.0f, 0.0f, 0.0f);
        ct::Vector<JointSetting> joints;

        bool enableAllChildCollisions = true;
        ct::Vector<ct::String> collisions;
        ct::Vector<ct::String> excludeCollisions;
        ct::Vector<SpringBoneCollision3D*> cachedCollisions;

        bool simulationDirty = false;
        Math::Mat4 cachedCenter;
        Math::Mat4 cachedInvertedCenter;
    };

    SpringBoneSimulator3D();

    bool hasProcess() const override;
    void restUpdated() override;

    void setRootBoneName(int index, const ct::String& boneName);
    const ct::String& getRootBoneName(int index) const;
    void setRootBone(int index, int bone);
    int getRootBone(int index) const;

    void setEndBoneName(int index, const ct::String& boneName);
    const ct::String& getEndBoneName(int index) const;
    void setEndBone(int index, int bone);
    int getEndBone(int index) const;

    void setExtendEndBone(int index, bool enabled);
    bool isEndBoneExtended(int index) const;
    void setEndBoneDirection(int index, BoneDirection boneDirection);
    BoneDirection getEndBoneDirection(int index) const;
    void setEndBoneLength(int index, float length);
    float getEndBoneLength(int index) const;
    Math::Vec3 getEndBoneAxis(int endBone, BoneDirection direction) const;

    void setCenterFrom(int index, CenterFrom centerFrom);
    CenterFrom getCenterFrom(int index) const;
    void setCenterNode(int index, const ct::String& nodePath);
    const ct::String& getCenterNode(int index) const;
    void setCenterBoneName(int index, const ct::String& boneName);
    const ct::String& getCenterBoneName(int index) const;
    void setCenterBone(int index, int bone);
    int getCenterBone(int index) const;

    void setRotationAxis(int index, RotationAxis axis);
    RotationAxis getRotationAxis(int index) const;
    void setRotationAxisVector(int index, const Math::Vec3& vector);
    Math::Vec3 getRotationAxisVector(int index) const;
    void setRadius(int index, float radius);
    float getRadius(int index) const;
    void setRadiusDampingCurve(int index, const ct::Vector<float>& dampingCurve);
    const ct::Vector<float>& getRadiusDampingCurve(int index) const;
    void setStiffness(int index, float stiffness);
    float getStiffness(int index) const;
    void setStiffnessDampingCurve(int index, const ct::Vector<float>& dampingCurve);
    const ct::Vector<float>& getStiffnessDampingCurve(int index) const;
    void setDrag(int index, float drag);
    float getDrag(int index) const;
    void setDragDampingCurve(int index, const ct::Vector<float>& dampingCurve);
    const ct::Vector<float>& getDragDampingCurve(int index) const;
    void setGravity(int index, float gravity);
    float getGravity(int index) const;
    void setGravityDampingCurve(int index, const ct::Vector<float>& dampingCurve);
    const ct::Vector<float>& getGravityDampingCurve(int index) const;
    void setGravityDirection(int index, const Math::Vec3& gravityDirection);
    Math::Vec3 getGravityDirection(int index) const;

    void setSettingCount(int count);
    int getSettingCount() const;
    void clearSettings();
    int addSetting(const ct::String& rootBoneName, const ct::String& endBoneName);

    void setIndividualConfig(int index, bool enabled);
    bool isConfigIndividual(int index) const;

    const ct::String& getJointBoneName(int index, int joint) const;
    int getJointBone(int index, int joint) const;

    void setJointRotationAxis(int index, int joint, RotationAxis axis);
    RotationAxis getJointRotationAxis(int index, int joint) const;
    void setJointRotationAxisVector(int index, int joint, const Math::Vec3& vector);
    Math::Vec3 getJointRotationAxisVector(int index, int joint) const;
    void setJointRadius(int index, int joint, float radius);
    float getJointRadius(int index, int joint) const;
    void setJointStiffness(int index, int joint, float stiffness);
    float getJointStiffness(int index, int joint) const;
    void setJointDrag(int index, int joint, float drag);
    float getJointDrag(int index, int joint) const;
    void setJointGravity(int index, int joint, float gravity);
    float getJointGravity(int index, int joint) const;
    void setJointGravityDirection(int index, int joint, const Math::Vec3& gravityDirection);
    Math::Vec3 getJointGravityDirection(int index, int joint) const;

    void setJointCount(int index, int count);
    int getJointCount(int index) const;

    void setEnableAllChildCollisions(int index, bool enabled);
    bool areAllChildCollisionsEnabled(int index) const;

    void setExcludeCollisionPath(int index, int collision, const ct::String& nodePath);
    const ct::String& getExcludeCollisionPath(int index, int collision) const;
    void setExcludeCollisionCount(int index, int count);
    int getExcludeCollisionCount(int index) const;
    void clearExcludeCollisions(int index);

    void setCollisionPath(int index, int collision, const ct::String& nodePath);
    const ct::String& getCollisionPath(int index, int collision) const;
    void setCollisionCount(int index, int count);
    int getCollisionCount(int index) const;
    void clearCollisions(int index);

    ct::Vector<SpringBoneCollision3D*> getValidCollisions(int index);

    void setExternalForce(const Math::Vec3& force);
    Math::Vec3 getExternalForce() const;

    void setMutableBoneAxes(bool enabled);
    bool areBoneAxesMutable() const;

    void reset();

    Math::Vec3 getBoneVector(int index, int joint) const;

protected:
    void notificationHandler(int what) override;
    void addChildNotify(Node* child) override;
    void removeChildNotify(Node* child) override;
    void moveChildNotify(Node* child) override;

    void validateBoneNames() override;
    void skeletonChanged(Skeleton3D* oldSkeleton, Skeleton3D* newSkeleton) override;
    void activeChanged(bool active) override;
    void processModificationImpl(double delta) override;

private:
    bool validIndex(int index) const;
    bool validJoint(int index, int joint) const;

    void findCollisions();
    void processCollisions();
    void makeCollisionsDirty();

    void initJoints(Skeleton3D* skeleton, Setting& setting);
    void processJoints(double delta, Skeleton3D* skeleton, ct::Vector<JointSetting>& joints,
            const ct::Vector<SpringBoneCollision3D*>& collisions, const Math::Mat4& centerTransform,
            const Math::Mat4& invertedCenterTransform,
            const Math::Quaternion& invertedCenterRotation);

    void makeJointsDirty(int index, bool reset = false);
    void makeAllJointsDirty();

    void updateJointArray(int index);
    void updateJoints();
    void setJointBone(int index, int joint, int bone);

    void updateBoneAxis(Skeleton3D* skeleton, Setting& setting);

    void validateRotationAxes(Skeleton3D* skeleton) const;
    void validateRotationAxis(Skeleton3D* skeleton, int index, int joint) const;

    ct::Vector<Setting> mSettings;
    Math::Vec3 mExternalForce;
    bool mMutableBoneAxes;

    bool mJointsDirty;
    bool mJointsReset;

    ct::Vector<SpringBoneCollision3D*> mCollisions;
    bool mCollisionsDirty;
};

} // namespace prisma3d
