#include "prisma3d/scene/BoneAttachment3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Skeleton3D.h"

#include "platform.h"

namespace prisma3d
{

BoneAttachment3D::BoneAttachment3D()
        : Node3D(),
          mBoundSkeleton(nullptr),
          mBoneName(),
          mBoneIdx(-1),
          mOverridePose(false),
          mOverrideDirty(false),
          mOverriding(false),
          mUseExternalSkeleton(false),
          mExternalSkeleton(),
          mUpdating(false)
{
}

BoneAttachment3D::~BoneAttachment3D()
{
    checkUnbind();
}

Skeleton3D* BoneAttachment3D::getSkeleton()
{
    if (mUseExternalSkeleton) return resolveExternalSkeleton();
    Node* parent = getParent();
    return parent ? parent->castTo<Skeleton3D>() : nullptr;
}

void BoneAttachment3D::setBoneName(const ct::String& name)
{
    mBoneName = name;
    if (Skeleton3D* skeleton = getSkeleton()) setBoneIdx(skeleton->findBone(mBoneName.c_str()));
}

const ct::String& BoneAttachment3D::getBoneName() const
{
    return mBoneName;
}

void BoneAttachment3D::setBoneIdx(int bone)
{
    if (isInsideTree()) checkUnbind();

    mBoneIdx = bone;

    if (Skeleton3D* skeleton = getSkeleton())
    {
        if (mBoneIdx <= -1 || static_cast<std::uint32_t>(mBoneIdx) >= skeleton->getBoneCount())
        {
            log_error("bone attachment: bone index %d out of range", mBoneIdx);
            mBoneIdx = -1;
        }
        else
        {
            mBoneName = skeleton->getBoneName(mBoneIdx);
        }
    }

    if (isInsideTree()) checkBind();
    else
        onSkeletonUpdate();
}

int BoneAttachment3D::getBoneIdx() const
{
    return mBoneIdx;
}

void BoneAttachment3D::setOverridePose(bool overridePose)
{
    if (mOverridePose == overridePose) return;

    mOverridePose = overridePose;
    setNotifyTransform(mOverridePose);
    setProcess(mOverridePose);
    if (!mOverridePose && mBoneIdx >= 0)
    {
        if (Skeleton3D* skeleton = getSkeleton()) skeleton->resetBonePose(mBoneIdx);
    }
}

bool BoneAttachment3D::getOverridePose() const
{
    return mOverridePose;
}

void BoneAttachment3D::setUseExternalSkeleton(bool useExternalSkeleton)
{
    if (isInsideTree()) checkUnbind();

    mUseExternalSkeleton = useExternalSkeleton;

    if (isInsideTree())
    {
        checkBind();
        if (mUseExternalSkeleton) transformChanged();
    }
    notifyPropertyListChanged();
}

bool BoneAttachment3D::getUseExternalSkeleton() const
{
    return mUseExternalSkeleton;
}

void BoneAttachment3D::setExternalSkeleton(const ct::String& path)
{
    if (isInsideTree()) checkUnbind();
    mExternalSkeleton = path;
    if (isInsideTree()) checkBind();
    notifyPropertyListChanged();
}

const ct::String& BoneAttachment3D::getExternalSkeleton() const
{
    return mExternalSkeleton;
}

void BoneAttachment3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name == "bone_name")
    {
        const Skeleton3D* skeleton = nullptr;
        if (mUseExternalSkeleton)
        {
            const Node* node = mExternalSkeleton.empty() ? nullptr : getNode(mExternalSkeleton);
            skeleton = node ? node->castTo<Skeleton3D>() : nullptr;
        }
        else
        {
            const Node* parent = getParent();
            skeleton = parent ? parent->castTo<Skeleton3D>() : nullptr;
        }

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
        return;
    }

    if (info.name == "external_skeleton" && !mUseExternalSkeleton) info.usage = PropertyUsage::None;
}

void BoneAttachment3D::onSkeletonUpdate()
{
    if (mUpdating) return;
    mUpdating = true;
    if (mBoneIdx >= 0)
    {
        if (Skeleton3D* skeleton = getSkeleton())
        {
            if (!mOverridePose)
            {
                if (mUseExternalSkeleton)
                {
                    if (skeleton->isInsideTree())
                        setGlobalTransform(skeleton->getGlobalTransform() *
                                           skeleton->getBoneGlobalPose(mBoneIdx));
                }
                else
                {
                    setTransform(skeleton->getBoneGlobalPose(mBoneIdx));
                }
            }
            else if (!mOverrideDirty)
            {
                transformChanged();
                mOverrideDirty = true;
            }
        }
    }
    mUpdating = false;
}

void BoneAttachment3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            checkBind();
            break;
        case kNotificationExitTree:
            checkUnbind();
            break;
        case kNotificationTransformChanged:
            transformChanged();
            break;
        case kNotificationProcess:
            mOverrideDirty = false;
            break;
        default:
            break;
    }
}

Skeleton3D* BoneAttachment3D::resolveExternalSkeleton()
{
    if (!mExternalSkeleton.empty())
    {
        Node* node = getNode(mExternalSkeleton);
        return node ? node->castTo<Skeleton3D>() : nullptr;
    }

    Node* parent = getParent();
    BoneAttachment3D* parentAttachment = parent ? parent->castTo<BoneAttachment3D>() : nullptr;
    if (!parentAttachment) return nullptr;

    Skeleton3D* skeleton = parentAttachment->resolveExternalSkeleton();
    if (!skeleton) return nullptr;

    const ct::String& parentPath = parentAttachment->mExternalSkeleton;
    mExternalSkeleton = parentPath.front() == '/' ? parentPath : "../" + parentPath;
    return skeleton;
}

void BoneAttachment3D::checkBind()
{
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton || mBoundSkeleton) return;

    if (mBoneIdx <= -1) mBoneIdx = skeleton->findBone(mBoneName.c_str());
    if (mBoneIdx == -1) return;

    skeleton->bindAttachment(this);
    mBoundSkeleton = skeleton;
    onSkeletonUpdate();
}

void BoneAttachment3D::checkUnbind()
{
    if (!mBoundSkeleton) return;
    mBoundSkeleton->unbindAttachment(this);
    mBoundSkeleton = nullptr;
}

void BoneAttachment3D::transformChanged()
{
    if (!isInsideTree()) return;

    if (mOverridePose && !mOverriding)
    {
        Skeleton3D* skeleton = getSkeleton();
        if (!skeleton)
        {
            log_error("bone attachment: cannot override pose: skeleton not found");
            return;
        }
        if (mBoneIdx < 0 || static_cast<std::uint32_t>(mBoneIdx) >= skeleton->getBoneCount())
        {
            log_error("bone attachment: cannot override pose: bone index is out of range");
            return;
        }

        Math::Mat4 pose = getTransform();
        if (mUseExternalSkeleton)
        {
            if (!skeleton->isInsideTree()) return;
            pose = skeleton->getGlobalTransform().InverseAffine() * getGlobalTransform();
        }

        mOverriding = true;
        skeleton->setBoneGlobalPose(mBoneIdx, pose);
        skeleton->forceUpdateAllBoneTransforms();
    }
    mOverriding = false;
}

void BoneAttachment3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = BoneAttachment3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "bone_name"), &P::setBoneName,
            &P::getBoneName);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "bone_idx"), &P::setBoneIdx, &P::getBoneIdx);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "override_pose"), &P::setOverridePose,
            &P::getOverridePose);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_external_skeleton"),
            &P::setUseExternalSkeleton, &P::getUseExternalSkeleton);
    DB::addProperty<P>(PropertyInfo::nodePath("external_skeleton", "Skeleton3D"),
            &P::setExternalSkeleton, &P::getExternalSkeleton);
}

} // namespace prisma3d
