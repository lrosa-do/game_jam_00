#include "prisma3d/scene/Skeleton3D.h"

#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SkeletonModifier3D.h"

#include <ct/json.hpp>

namespace prisma3d
{

namespace
{

const ct::String kEmptyName;

bool validBone(const Skeleton& skeleton, int bone)
{
    return bone >= 0 && static_cast<std::uint32_t>(bone) < skeleton.boneCount();
}

bool samePose(const LocalPose& a, const LocalPose& b)
{
    return a.position.x == b.position.x && a.position.y == b.position.y &&
           a.position.z == b.position.z && a.rotation.x == b.rotation.x &&
           a.rotation.y == b.rotation.y && a.rotation.z == b.rotation.z &&
           a.rotation.w == b.rotation.w && a.scale.x == b.scale.x && a.scale.y == b.scale.y &&
           a.scale.z == b.scale.z;
}

LocalPose interpolatePose(const LocalPose& from, const LocalPose& to, float weight)
{
    LocalPose result;
    result.position = Math::Vec3::Lerp(from.position, to.position, weight);
    result.rotation = Math::Quaternion::Slerp(from.rotation, to.rotation, weight).Normalized();
    result.scale = Math::Vec3::Lerp(from.scale, to.scale, weight);
    return result;
}

} // namespace

Skeleton3D::Skeleton3D()
        : Node3D(),
          mSkeleton(),
          mRest(),
          mPose(),
          mGlobalPose(),
          mPalette(),
          mServerSkeleton(),
          mPoseVersion(0),
          mDirty(false),
          mGlobalDirty(true),
          mUpdating(false),
          mAttachments(),
          mModifierCallbackModeProcess(ModifierCallbackModeProcess::Physics),
          mModifiers(),
          mModifiersDirty(true),
          mModifierUpdating(false),
          mDestroyed(false),
          mUpdateDelta(0.0),
          mPoseBackup(),
          mModifierBefore(),
          mModifierApplied()
{

    if (RenderingServer* server = RenderingServer::singleton())
        mServerSkeleton = server->skeletonCreate();
}

Skeleton3D::~Skeleton3D()
{
    mDestroyed = true;
    for (std::size_t i = 0; i < mAttachments.size(); ++i)
        if (mAttachments[i]) mAttachments[i]->mBoundSkeleton = nullptr;

    SceneTree* tree = getTree() ? getTree() : SceneTree::singleton();
    if (tree) tree->dropSkeletonUpdate(this);

    if (RenderingServer* server = RenderingServer::singleton())
        server->skeletonFree(mServerSkeleton);
}

SkeletonId Skeleton3D::getSkeletonId() const
{
    return mServerSkeleton;
}

void Skeleton3D::setSkeleton(const Skeleton& skeleton)
{
    mSkeleton = skeleton;
    if (!mSkeleton.finalized()) mSkeleton.finalize();
    mSkeleton.bindPose(mRest);
    mPose = mRest;
    mGlobalPose.clear();
    mPalette.clear();
    mGlobalDirty = true;

    if (RenderingServer* server = RenderingServer::singleton())
        server->skeletonAllocateData(mServerSkeleton, mSkeleton.boneCount());
    makeDirty();

    findModifiers();
    for (std::size_t i = 0; i < mModifiers.size(); ++i) mModifiers[i]->restUpdated();
}

const Skeleton& Skeleton3D::getSkeleton() const
{
    return mSkeleton;
}

void Skeleton3D::clearBones()
{
    setSkeleton(Skeleton());
}

std::uint32_t Skeleton3D::getBoneCount() const
{
    return mSkeleton.boneCount();
}

int Skeleton3D::findBone(const char* name) const
{
    return mSkeleton.findBone(name);
}

const ct::String& Skeleton3D::getBoneName(int bone) const
{
    if (!validBone(mSkeleton, bone)) return kEmptyName;
    return mSkeleton.bone(static_cast<std::uint32_t>(bone)).name;
}

int Skeleton3D::getBoneParent(int bone) const
{
    if (!validBone(mSkeleton, bone)) return -1;
    return mSkeleton.bone(static_cast<std::uint32_t>(bone)).parent;
}

Math::Mat4 Skeleton3D::getBoneRest(int bone) const
{
    if (!validBone(mSkeleton, bone)) return Math::Mat4::Identity();
    return mSkeleton.bone(static_cast<std::uint32_t>(bone)).bindLocal;
}

const LocalPose& Skeleton3D::getBoneRestPose(int bone) const
{
    static const LocalPose identity;
    if (!validBone(mSkeleton, bone) || static_cast<std::size_t>(bone) >= mRest.size())
        return identity;
    return mRest[static_cast<std::size_t>(bone)];
}

Math::Mat4 Skeleton3D::getBoneGlobalRest(int bone) const
{
    if (!validBone(mSkeleton, bone)) return Math::Mat4::Identity();
    Math::Mat4 result = mSkeleton.bone(static_cast<std::uint32_t>(bone)).bindLocal;
    int parent = mSkeleton.bone(static_cast<std::uint32_t>(bone)).parent;
    std::uint32_t guard = 0;
    while (validBone(mSkeleton, parent) && guard++ < mSkeleton.boneCount())
    {
        const Bone& parentBone = mSkeleton.bone(static_cast<std::uint32_t>(parent));
        result = parentBone.bindLocal * result;
        parent = parentBone.parent;
    }
    return result;
}

void Skeleton3D::setBonePose(int bone, const LocalPose& pose)
{
    if (!validBone(mSkeleton, bone)) return;
    if (mModifierUpdating) markBoneModified(bone);
    mPose[static_cast<std::size_t>(bone)] = pose;
    mGlobalDirty = true;
    makeDirty();
}

const LocalPose& Skeleton3D::getBonePose(int bone) const
{
    static const LocalPose identity;
    if (!validBone(mSkeleton, bone)) return identity;
    return mPose[static_cast<std::size_t>(bone)];
}

void Skeleton3D::setBonePosePosition(int bone, const Math::Vec3& position)
{
    if (!validBone(mSkeleton, bone)) return;
    if (mModifierUpdating) markBoneModified(bone);
    mPose[static_cast<std::size_t>(bone)].position = position;
    mGlobalDirty = true;
    makeDirty();
}

void Skeleton3D::setBonePoseRotation(int bone, const Math::Quaternion& rotation)
{
    if (!validBone(mSkeleton, bone)) return;
    if (mModifierUpdating) markBoneModified(bone);
    mPose[static_cast<std::size_t>(bone)].rotation = rotation;
    mGlobalDirty = true;
    makeDirty();
}

void Skeleton3D::setBonePoseScale(int bone, const Math::Vec3& scale)
{
    if (!validBone(mSkeleton, bone)) return;
    if (mModifierUpdating) markBoneModified(bone);
    mPose[static_cast<std::size_t>(bone)].scale = scale;
    mGlobalDirty = true;
    makeDirty();
}

Math::Vec3 Skeleton3D::getBonePosePosition(int bone) const
{
    return getBonePose(bone).position;
}

Math::Quaternion Skeleton3D::getBonePoseRotation(int bone) const
{
    return getBonePose(bone).rotation;
}

Math::Vec3 Skeleton3D::getBonePoseScale(int bone) const
{
    return getBonePose(bone).scale;
}

void Skeleton3D::resetBonePose(int bone)
{
    if (!validBone(mSkeleton, bone)) return;
    setBonePose(bone, mRest[static_cast<std::size_t>(bone)]);
}

void Skeleton3D::resetBonePoses()
{
    mPose = mRest;
    mGlobalDirty = true;
    makeDirty();
}

Math::Mat4 Skeleton3D::getBoneGlobalPose(int bone)
{
    if (!validBone(mSkeleton, bone)) return Math::Mat4::Identity();
    if (mGlobalDirty) forceUpdateAllBoneTransforms();
    if (static_cast<std::size_t>(bone) >= mGlobalPose.size()) return Math::Mat4::Identity();
    return mGlobalPose[static_cast<std::size_t>(bone)];
}

void Skeleton3D::setBoneGlobalPose(int bone, const Math::Mat4& pose)
{
    if (!validBone(mSkeleton, bone)) return;
    const int parent = mSkeleton.bone(static_cast<std::uint32_t>(bone)).parent;
    const Math::Mat4 parentGlobal =
            parent >= 0 ? getBoneGlobalPose(parent) : Math::Mat4::Identity();
    LocalPose local;
    if (!LocalPose::fromMatrix(parentGlobal.InverseAffine() * pose, local)) return;
    setBonePose(bone, local);
}

const ct::Vector<Math::Mat4>& Skeleton3D::getBonePalette()
{
    if (mGlobalDirty) forceUpdateAllBoneTransforms();
    return mPalette;
}

void Skeleton3D::forceUpdateAllBoneTransforms()
{
    if (mSkeleton.empty())
    {
        mGlobalPose.clear();
        mPalette.clear();
    }
    else
    {
        mSkeleton.evaluate(mPose, mGlobalPose, mPalette);
    }
    mGlobalDirty = false;
}

void Skeleton3D::forceUpdateDeferred()
{
    if (mDestroyed) return;
    makeDirty();
}

void Skeleton3D::updateSkeleton()
{
    if (!mDirty) return;
    forceUpdateAllBoneTransforms();

    mUpdating = true;

    findModifiers();
    const bool modified = !mModifiers.empty();
    if (modified)
    {
        mPoseBackup = mPose;
        processModifiers();
    }

    if (poseUpdated) poseUpdated();
    notifyAttachments();
    if (mGlobalDirty) forceUpdateAllBoneTransforms();

    pushToServer();
    ++mPoseVersion;

    if (modified)
    {
        mPose = mPoseBackup;
        mGlobalDirty = true;
    }

    mDirty = false;
    mUpdating = false;
}

bool Skeleton3D::isDirty() const
{
    return mDirty;
}

std::uint64_t Skeleton3D::getPoseVersion() const
{
    return mPoseVersion;
}

void Skeleton3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            if (mModifierCallbackModeProcess == ModifierCallbackModeProcess::Idle) setProcess(true);
            else if (mModifierCallbackModeProcess == ModifierCallbackModeProcess::Physics)
                setPhysicsProcess(true);
            break;
        case kNotificationPostEnterTree:
            mDirty = true;
            mModifiersDirty = true;
            if (SceneTree* tree = getTree()) tree->queueSkeletonUpdate(this);
            break;
        case kNotificationExitTree:
            if (SceneTree* tree = getTree()) tree->dropSkeletonUpdate(this);
            break;
        case kNotificationProcess:
            advance(getProcessDeltaTime());
            break;
        case kNotificationPhysicsProcess:
            advance(getPhysicsProcessDeltaTime());
            break;
        default:
            break;
    }
}

void Skeleton3D::addChildNotify(Node* child)
{
    Node3D::addChildNotify(child);
    makeModifiersDirty();
    if (child->castTo<SkeletonModifier3D>()) makeDirty();
}

void Skeleton3D::removeChildNotify(Node* child)
{
    Node3D::removeChildNotify(child);
    makeModifiersDirty();
    if (child->castTo<SkeletonModifier3D>()) makeDirty();
}

void Skeleton3D::moveChildNotify(Node* child)
{
    Node3D::moveChildNotify(child);
    makeModifiersDirty();
    if (child->castTo<SkeletonModifier3D>()) makeDirty();
}

void Skeleton3D::setModifierCallbackModeProcess(ModifierCallbackModeProcess mode)
{
    if (mModifierCallbackModeProcess == mode) return;
    mModifierCallbackModeProcess = mode;
    processChanged();
}

Skeleton3D::ModifierCallbackModeProcess Skeleton3D::getModifierCallbackModeProcess() const
{
    return mModifierCallbackModeProcess;
}

void Skeleton3D::processChanged()
{
    setProcess(mModifierCallbackModeProcess == ModifierCallbackModeProcess::Idle);
    setPhysicsProcess(mModifierCallbackModeProcess == ModifierCallbackModeProcess::Physics);
}

void Skeleton3D::advance(double delta)
{
    findModifiers();
    if (!mModifiers.empty())
    {
        mUpdateDelta += delta;
        makeDirty();
    }
}

void Skeleton3D::makeModifiersDirty()
{
    mModifiersDirty = true;
}

void Skeleton3D::findModifiers()
{
    if (!mModifiersDirty) return;
    mModifiers.clear();
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        if (SkeletonModifier3D* modifier = getChild(i)->castTo<SkeletonModifier3D>())
            mModifiers.push_back(modifier);
    }
    mModifiersDirty = false;
}

void Skeleton3D::markBoneModified(int bone)
{
    const std::size_t index = static_cast<std::size_t>(bone);
    if (index >= mModifierApplied.size() || mModifierApplied[index]) return;
    mModifierApplied[index] = 1;
    mModifierBefore[index] = mPose[index];
}

void Skeleton3D::processModifiers()
{
    for (std::size_t m = 0; m < mModifiers.size(); ++m)
    {
        SkeletonModifier3D* modifier = mModifiers[m];
        const float influence = modifier->getInfluence();
        if (influence < 1.0f)
        {
            mModifierApplied.assign(mPose.size(), static_cast<std::uint8_t>(0));
            mModifierBefore.resize(mPose.size());
            mModifierUpdating = true;
            modifier->processModification(mUpdateDelta);
            mModifierUpdating = false;
            for (std::size_t i = 0; i < mPose.size(); ++i)
            {
                if (!mModifierApplied[i]) continue;
                mModifierApplied[i] = 0;
                if (samePose(mPose[i], mModifierBefore[i])) continue;
                setBonePose(static_cast<int>(i),
                        interpolatePose(mModifierBefore[i], mPose[i], influence));
            }
        }
        else
        {
            modifier->processModification(mUpdateDelta);
        }
        forceUpdateAllBoneTransforms();
    }
    mUpdateDelta = 0.0;
}

void Skeleton3D::makeDirty()
{
    if (mUpdating) return;
    mDirty = true;
    if (!isInsideTree()) return;
    if (SceneTree* tree = getTree()) tree->queueSkeletonUpdate(this);
}

void Skeleton3D::pushToServer()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || !mServerSkeleton.valid()) return;
    const std::uint32_t bones = static_cast<std::uint32_t>(mPalette.size());
    if (server->skeletonGetBoneCount(mServerSkeleton) != bones)
        server->skeletonAllocateData(mServerSkeleton, bones);
    for (std::uint32_t i = 0; i < bones; ++i)
        server->skeletonBoneSetTransform(mServerSkeleton, i, mPalette[i]);
}

void Skeleton3D::bindAttachment(BoneAttachment3D* attachment)
{
    for (std::size_t i = 0; i < mAttachments.size(); ++i)
        if (mAttachments[i] == attachment) return;
    mAttachments.push_back(attachment);
}

void Skeleton3D::unbindAttachment(BoneAttachment3D* attachment)
{
    for (std::size_t i = 0; i < mAttachments.size(); ++i)
        if (mAttachments[i] == attachment) mAttachments[i] = nullptr;
}

void Skeleton3D::notifyAttachments()
{
    std::size_t kept = 0;
    for (std::size_t i = 0; i < mAttachments.size(); ++i)
    {
        BoneAttachment3D* attachment = mAttachments[i];
        if (!attachment) continue;
        attachment->onSkeletonUpdate();
        if (mAttachments[i]) mAttachments[kept++] = attachment;
    }
    mAttachments.resize(kept);
}

void Skeleton3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Skeleton3D;
    DB::addProperty<P>(
            PropertyInfo::enumeration("modifier_callback_mode_process", "Physics,Idle,Manual"),
            &P::setModifierCallbackModeProcess, &P::getModifierCallbackModeProcess);
}

namespace
{

constexpr std::int64_t kSkeleton3DDataVersion = 1;

}

bool Skeleton3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mSkeleton.boneCount() == 0) return false;

    data["version"] = ct::Json(static_cast<long long>(kSkeleton3DDataVersion));
    ct::Json bones = ct::Json::array();
    for (std::uint32_t i = 0; i < mSkeleton.boneCount(); ++i)
    {
        const Bone& bone = mSkeleton.bone(i);
        ct::Json entry = ct::Json::object();
        entry["name"] = ct::Json(bone.name);
        entry["parent"] = ct::Json(static_cast<long long>(bone.parent));
        entry["bind"] = scenejson::fromMat4(bone.bindLocal);
        entry["inverse_bind"] = scenejson::fromMat4(bone.inverseBind);
        bones.push_back(entry);
    }
    data["bones"] = bones;

    ct::Json poses = ct::Json::array();
    for (std::size_t i = 0; i < mPose.size() && i < mRest.size(); ++i)
    {
        const LocalPose& pose = mPose[i];
        const LocalPose& rest = mRest[i];
        if (scenejson::exact(pose.position, rest.position) &&
                scenejson::exact(pose.rotation, rest.rotation) &&
                scenejson::exact(pose.scale, rest.scale))
            continue;
        ct::Json entry = ct::Json::object();
        entry["bone"] = ct::Json(static_cast<long long>(i));
        entry["position"] = scenejson::fromVec3(pose.position);
        entry["rotation"] = scenejson::fromQuat(pose.rotation);
        entry["scale"] = scenejson::fromVec3(pose.scale);
        poses.push_back(entry);
    }
    if (poses.size() > 0) data["poses"] = poses;
    return true;
}

bool Skeleton3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kSkeleton3DDataVersion, result, path)) return false;

    Skeleton skeleton;
    if (const ct::Json* bones = scenejson::arrayField(data, "bones", result, path))
    {
        for (std::size_t i = 0; i < bones->size(); ++i)
        {
            const ct::Json& entry = bones->at(i);
            const ct::String bonePath = scenejson::element(path, "bones", i);
            if (!entry.is_object())
            {
                result.addError(bonePath, "expected an object");
                return false;
            }
            const ct::String name = scenejson::getString(entry, "name", ct::String());
            const int parent = static_cast<int>(scenejson::getInt(entry, "parent", -1));
            Math::Mat4 bind = Math::Mat4::Identity();
            Math::Mat4 inverseBind = Math::Mat4::Identity();
            const ct::Json* bindField = entry.find("bind");
            const ct::Json* inverseField = entry.find("inverse_bind");
            if ((bindField && !scenejson::toMat4(*bindField, bind)) ||
                    (inverseField && !scenejson::toMat4(*inverseField, inverseBind)))
            {
                result.addError(bonePath, "the matrices must be 16 numbers");
                return false;
            }
            if (!skeleton.addBone(name, parent, bind, inverseBind))
            {
                result.addError(bonePath, "the bone has no name, a repeated name or a bad parent");
                return false;
            }
        }
    }
    if (skeleton.boneCount() > 0 && !skeleton.finalize())
    {
        result.addError(scenejson::field(path, "bones"), "the bone parents form a cycle");
        return false;
    }
    setSkeleton(skeleton);

    if (const ct::Json* poses = scenejson::arrayField(data, "poses", result, path))
    {
        for (std::size_t i = 0; i < poses->size(); ++i)
        {
            const ct::Json& entry = poses->at(i);
            const ct::String posePath = scenejson::element(path, "poses", i);
            const std::int64_t bone = entry.is_object() ? scenejson::getInt(entry, "bone", -1) : -1;
            if (bone < 0 || bone >= static_cast<std::int64_t>(getBoneCount()))
            {
                result.addError(posePath, "the bone index is missing or out of range");
                continue;
            }
            LocalPose pose = getBoneRestPose(static_cast<int>(bone));
            scenejson::readVec3Field(entry, "position", pose.position, result, posePath);
            const ct::Json* rotation = entry.find("rotation");
            if (rotation && !scenejson::toQuat(*rotation, pose.rotation))
                result.addError(scenejson::field(posePath, "rotation"),
                        "expected [x, y, z, w] of numbers");
            scenejson::readVec3Field(entry, "scale", pose.scale, result, posePath);
            setBonePose(static_cast<int>(bone), pose);
        }
    }
    return true;
}

} // namespace prisma3d
