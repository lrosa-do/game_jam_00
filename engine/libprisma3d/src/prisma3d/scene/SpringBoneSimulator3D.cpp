#include "prisma3d/scene/SpringBoneSimulator3D.h"

#include "ModifierMath.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SpringBoneCollision3D.h"

#include "platform.h"

#include <ct/json.hpp>

#include <cmath>

namespace prisma3d
{

namespace
{

constexpr std::int64_t kSpringBoneDataVersion = 1;

const ct::String kEmptyString;
const ct::Vector<float> kEmptyCurve;

using modifiermath::normalized;

Math::Vec3 rotateBy(const Math::Quaternion& rotation, const Math::Vec3& v)
{
    return rotation * v;
}

Math::Vec3 transformPoint(const Math::Mat4& transform, const Math::Vec3& v)
{
    return transform.TransformPoint(v);
}

} // namespace

float SpringBoneSimulator3D::DampingCurve::sampleBaked(float offset) const
{
    if (!std::isfinite(offset))
    {
        log_error("spring bone: damping curve offset is non-finite");
        return 0.0f;
    }

    const int count = static_cast<int>(samples.size());
    if (count == 0) return 0.0f;
    if (count == 1) return samples[0];

    float fi = offset * static_cast<float>(count - 1);
    int i = static_cast<int>(std::floor(fi));
    if (i < 0)
    {
        i = 0;
        fi = 0.0f;
    }
    else if (i >= count)
    {
        i = count - 1;
        fi = 0.0f;
    }

    if (i + 1 < count)
    {
        const float t = fi - static_cast<float>(i);
        return samples[static_cast<std::size_t>(i)] +
               (samples[static_cast<std::size_t>(i + 1)] - samples[static_cast<std::size_t>(i)]) *
                       t;
    }
    return samples[static_cast<std::size_t>(count - 1)];
}

Math::Vec3 SpringBoneSimulator3D::JointSetting::getRotationAxisVector() const
{
    Math::Vec3 result;
    switch (rotationAxis)
    {
        case RotationAxis::X:
            result = Math::Vec3(1.0f, 0.0f, 0.0f);
            break;
        case RotationAxis::Y:
            result = Math::Vec3(0.0f, 1.0f, 0.0f);
            break;
        case RotationAxis::Z:
            result = Math::Vec3(0.0f, 0.0f, 1.0f);
            break;
        case RotationAxis::All:
            result = Math::Vec3(0.0f, 0.0f, 0.0f);
            break;
        case RotationAxis::Custom:
            result = rotationAxisVector;
            break;
    }
    return result;
}

SpringBoneSimulator3D::SpringBoneSimulator3D()
        : SkeletonModifier3D(),
          mSettings(),
          mExternalForce(0.0f, 0.0f, 0.0f),
          mMutableBoneAxes(true),
          mJointsDirty(false),
          mJointsReset(false),
          mCollisions(),
          mCollisionsDirty(false)
{
}

bool SpringBoneSimulator3D::hasProcess() const
{
    return true;
}

bool SpringBoneSimulator3D::validIndex(int index) const
{
    if (index >= 0 && static_cast<std::size_t>(index) < mSettings.size()) return true;
    log_error("spring bone: setting index %d is out of range", index);
    return false;
}

bool SpringBoneSimulator3D::validJoint(int index, int joint) const
{
    if (!validIndex(index)) return false;
    if (joint >= 0 && static_cast<std::size_t>(joint) < mSettings[index].joints.size()) return true;
    log_error("spring bone: joint index %d is out of range", joint);
    return false;
}

void SpringBoneSimulator3D::notificationHandler(int what)
{
    SkeletonModifier3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            makeCollisionsDirty();
            makeAllJointsDirty();
            break;
        default:
            break;
    }
}

void SpringBoneSimulator3D::setRootBoneName(int index, const ct::String& boneName)
{
    if (!validIndex(index)) return;
    mSettings[index].rootBoneName = boneName;
    Skeleton3D* sk = getSkeleton();
    if (sk) setRootBone(index, sk->findBone(mSettings[index].rootBoneName.c_str()));
}

const ct::String& SpringBoneSimulator3D::getRootBoneName(int index) const
{
    if (!validIndex(index)) return kEmptyString;
    return mSettings[index].rootBoneName;
}

void SpringBoneSimulator3D::setRootBone(int index, int bone)
{
    if (!validIndex(index)) return;
    const bool changed = mSettings[index].rootBone != bone;
    mSettings[index].rootBone = bone;
    Skeleton3D* sk = getSkeleton();
    if (sk)
    {
        if (mSettings[index].rootBone <= -1 ||
                static_cast<std::uint32_t>(mSettings[index].rootBone) >= sk->getBoneCount())
        {
            log_warn("spring bone: setting %d: root bone index '%d' is out of range", index, bone);
            mSettings[index].rootBone = -1;
        }
        else
        {
            mSettings[index].rootBoneName = sk->getBoneName(mSettings[index].rootBone);
        }
    }
    if (changed) updateJointArray(index);
}

int SpringBoneSimulator3D::getRootBone(int index) const
{
    if (!validIndex(index)) return -1;
    return mSettings[index].rootBone;
}

void SpringBoneSimulator3D::setEndBoneName(int index, const ct::String& boneName)
{
    if (!validIndex(index)) return;
    mSettings[index].endBoneName = boneName;
    Skeleton3D* sk = getSkeleton();
    if (sk) setEndBone(index, sk->findBone(mSettings[index].endBoneName.c_str()));
}

const ct::String& SpringBoneSimulator3D::getEndBoneName(int index) const
{
    if (!validIndex(index)) return kEmptyString;
    return mSettings[index].endBoneName;
}

void SpringBoneSimulator3D::setEndBone(int index, int bone)
{
    if (!validIndex(index)) return;
    const bool changed = mSettings[index].endBone != bone;
    mSettings[index].endBone = bone;
    Skeleton3D* sk = getSkeleton();
    if (sk)
    {
        if (mSettings[index].endBone <= -1 ||
                static_cast<std::uint32_t>(mSettings[index].endBone) >= sk->getBoneCount())
        {
            log_warn("spring bone: setting %d: end bone index '%d' is out of range", index, bone);
            mSettings[index].endBone = -1;
        }
        else
        {
            mSettings[index].endBoneName = sk->getBoneName(mSettings[index].endBone);
        }
    }
    if (changed) updateJointArray(index);
    notifyPropertyListChanged();
}

int SpringBoneSimulator3D::getEndBone(int index) const
{
    if (!validIndex(index)) return -1;
    return mSettings[index].endBone;
}

void SpringBoneSimulator3D::setExtendEndBone(int index, bool enabled)
{
    if (!validIndex(index)) return;
    mSettings[index].extendEndBone = enabled;
    makeJointsDirty(index, true);
    notifyPropertyListChanged();
}

bool SpringBoneSimulator3D::isEndBoneExtended(int index) const
{
    if (!validIndex(index)) return false;
    return mSettings[index].extendEndBone;
}

void SpringBoneSimulator3D::setEndBoneDirection(int index, BoneDirection boneDirection)
{
    if (!validIndex(index)) return;
    mSettings[index].endBoneDirection = boneDirection;
    if (mMutableBoneAxes) return;
    makeJointsDirty(index, true);
}

SkeletonModifier3D::BoneDirection SpringBoneSimulator3D::getEndBoneDirection(int index) const
{
    if (!validIndex(index)) return BoneDirection::FromParent;
    return mSettings[index].endBoneDirection;
}

void SpringBoneSimulator3D::setEndBoneLength(int index, float length)
{
    if (!validIndex(index)) return;
    const float old = mSettings[index].endBoneLength;
    mSettings[index].endBoneLength = length;
    if (mMutableBoneAxes && modifiermath::isZeroApprox(old) == modifiermath::isZeroApprox(length))
        return;
    makeJointsDirty(index, true);
}

float SpringBoneSimulator3D::getEndBoneLength(int index) const
{
    if (!validIndex(index)) return 0.0f;
    return mSettings[index].endBoneLength;
}

Math::Vec3 SpringBoneSimulator3D::getEndBoneAxis(int endBone, BoneDirection direction) const
{
    Math::Vec3 axis;
    if (direction == BoneDirection::FromParent)
    {
        Skeleton3D* sk = getSkeleton();
        if (sk)
        {
            const Math::Vec3 origin = mMutableBoneAxes ? sk->getBonePose(endBone).position
                                                       : sk->getBoneRest(endBone).GetTranslation();
            axis = modifiermath::xformInverse(sk->getBoneRest(endBone).UpperLeft3x3(), origin);
            axis = normalized(axis);
        }
    }
    else
    {
        axis = getVectorFromBoneAxis(static_cast<BoneAxis>(static_cast<std::uint8_t>(direction)));
    }
    return axis;
}

void SpringBoneSimulator3D::setCenterFrom(int index, CenterFrom centerFrom)
{
    if (!validIndex(index)) return;
    const bool centerChanged = mSettings[index].centerFrom != centerFrom;
    mSettings[index].centerFrom = centerFrom;
    if (centerChanged) reset();
    notifyPropertyListChanged();
}

SpringBoneSimulator3D::CenterFrom SpringBoneSimulator3D::getCenterFrom(int index) const
{
    if (!validIndex(index)) return CenterFrom::WorldOrigin;
    return mSettings[index].centerFrom;
}

void SpringBoneSimulator3D::setCenterNode(int index, const ct::String& nodePath)
{
    if (!validIndex(index)) return;
    const bool centerChanged = mSettings[index].centerNode != nodePath;
    if (isInsideTree() && !nodePath.empty())
    {
        Node* node = getNode(nodePath);
        if (!node || !node->castTo<Node3D>())
            log_warn("spring bone: setting %d: center node '%s' not found", index,
                    nodePath.c_str());
    }
    mSettings[index].centerNode = nodePath;
    if (centerChanged) reset();
}

const ct::String& SpringBoneSimulator3D::getCenterNode(int index) const
{
    if (!validIndex(index)) return kEmptyString;
    return mSettings[index].centerNode;
}

void SpringBoneSimulator3D::setCenterBoneName(int index, const ct::String& boneName)
{
    if (!validIndex(index)) return;
    mSettings[index].centerBoneName = boneName;
    Skeleton3D* sk = getSkeleton();
    if (sk) setCenterBone(index, sk->findBone(mSettings[index].centerBoneName.c_str()));
}

const ct::String& SpringBoneSimulator3D::getCenterBoneName(int index) const
{
    if (!validIndex(index)) return kEmptyString;
    return mSettings[index].centerBoneName;
}

void SpringBoneSimulator3D::setCenterBone(int index, int bone)
{
    if (!validIndex(index)) return;
    const bool centerChanged = mSettings[index].centerBone != bone;
    mSettings[index].centerBone = bone;
    Skeleton3D* sk = getSkeleton();
    if (sk)
    {
        if (mSettings[index].centerBone <= -1 ||
                static_cast<std::uint32_t>(mSettings[index].centerBone) >= sk->getBoneCount())
        {
            log_warn("spring bone: setting %d: center bone index '%d' is out of range", index,
                    bone);
            mSettings[index].centerBone = -1;
        }
        else
        {
            mSettings[index].centerBoneName = sk->getBoneName(mSettings[index].centerBone);
        }
    }
    if (centerChanged) reset();
}

int SpringBoneSimulator3D::getCenterBone(int index) const
{
    if (!validIndex(index)) return -1;
    return mSettings[index].centerBone;
}

void SpringBoneSimulator3D::setRadius(int index, float radius)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].radius = radius;
    makeJointsDirty(index);
}

float SpringBoneSimulator3D::getRadius(int index) const
{
    if (!validIndex(index)) return 0.0f;
    return mSettings[index].radius;
}

void SpringBoneSimulator3D::setRadiusDampingCurve(int index, const ct::Vector<float>& dampingCurve)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].radiusDampingCurve.samples = dampingCurve;
    makeJointsDirty(index);
}

const ct::Vector<float>& SpringBoneSimulator3D::getRadiusDampingCurve(int index) const
{
    if (!validIndex(index)) return kEmptyCurve;
    return mSettings[index].radiusDampingCurve.samples;
}

void SpringBoneSimulator3D::setStiffness(int index, float stiffness)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].stiffness = stiffness;
    makeJointsDirty(index);
}

float SpringBoneSimulator3D::getStiffness(int index) const
{
    if (!validIndex(index)) return 0.0f;
    return mSettings[index].stiffness;
}

void SpringBoneSimulator3D::setStiffnessDampingCurve(int index,
        const ct::Vector<float>& dampingCurve)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].stiffnessDampingCurve.samples = dampingCurve;
    makeJointsDirty(index);
}

const ct::Vector<float>& SpringBoneSimulator3D::getStiffnessDampingCurve(int index) const
{
    if (!validIndex(index)) return kEmptyCurve;
    return mSettings[index].stiffnessDampingCurve.samples;
}

void SpringBoneSimulator3D::setDrag(int index, float drag)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].drag = drag;
    makeJointsDirty(index);
}

float SpringBoneSimulator3D::getDrag(int index) const
{
    if (!validIndex(index)) return 0.0f;
    return mSettings[index].drag;
}

void SpringBoneSimulator3D::setDragDampingCurve(int index, const ct::Vector<float>& dampingCurve)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].dragDampingCurve.samples = dampingCurve;
    makeJointsDirty(index);
}

const ct::Vector<float>& SpringBoneSimulator3D::getDragDampingCurve(int index) const
{
    if (!validIndex(index)) return kEmptyCurve;
    return mSettings[index].dragDampingCurve.samples;
}

void SpringBoneSimulator3D::setGravity(int index, float gravity)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].gravity = gravity;
    makeJointsDirty(index);
}

float SpringBoneSimulator3D::getGravity(int index) const
{
    if (!validIndex(index)) return 0.0f;
    return mSettings[index].gravity;
}

void SpringBoneSimulator3D::setGravityDampingCurve(int index, const ct::Vector<float>& dampingCurve)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].gravityDampingCurve.samples = dampingCurve;
    makeJointsDirty(index);
}

const ct::Vector<float>& SpringBoneSimulator3D::getGravityDampingCurve(int index) const
{
    if (!validIndex(index)) return kEmptyCurve;
    return mSettings[index].gravityDampingCurve.samples;
}

void SpringBoneSimulator3D::setGravityDirection(int index, const Math::Vec3& gravityDirection)
{
    if (!validIndex(index)) return;
    if (modifiermath::isZeroApprox(gravityDirection))
    {
        log_error("spring bone: the gravity direction cannot be zero");
        return;
    }
    if (isConfigIndividual(index)) return;
    mSettings[index].gravityDirection = gravityDirection;
    makeJointsDirty(index);
}

Math::Vec3 SpringBoneSimulator3D::getGravityDirection(int index) const
{
    if (!validIndex(index)) return Math::Vec3(0.0f, -1.0f, 0.0f);
    return mSettings[index].gravityDirection;
}

void SpringBoneSimulator3D::setRotationAxis(int index, RotationAxis axis)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index)) return;
    mSettings[index].rotationAxis = axis;
    makeJointsDirty(index);
    notifyPropertyListChanged();
}

SkeletonModifier3D::RotationAxis SpringBoneSimulator3D::getRotationAxis(int index) const
{
    if (!validIndex(index)) return RotationAxis::All;
    return mSettings[index].rotationAxis;
}

void SpringBoneSimulator3D::setRotationAxisVector(int index, const Math::Vec3& vector)
{
    if (!validIndex(index)) return;
    if (isConfigIndividual(index) || mSettings[index].rotationAxis != RotationAxis::Custom) return;
    mSettings[index].rotationAxisVector = vector;
    makeJointsDirty(index);
}

Math::Vec3 SpringBoneSimulator3D::getRotationAxisVector(int index) const
{
    if (!validIndex(index)) return Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 result;
    switch (mSettings[index].rotationAxis)
    {
        case RotationAxis::X:
            result = Math::Vec3(1.0f, 0.0f, 0.0f);
            break;
        case RotationAxis::Y:
            result = Math::Vec3(0.0f, 1.0f, 0.0f);
            break;
        case RotationAxis::Z:
            result = Math::Vec3(0.0f, 0.0f, 1.0f);
            break;
        case RotationAxis::All:
            result = Math::Vec3(0.0f, 0.0f, 0.0f);
            break;
        case RotationAxis::Custom:
            result = mSettings[index].rotationAxisVector;
            break;
    }
    return result;
}

void SpringBoneSimulator3D::setSettingCount(int count)
{
    if (count < 0)
    {
        log_error("spring bone: the setting count cannot be negative");
        return;
    }
    mSettings.resize(static_cast<std::size_t>(count));
    makeCollisionsDirty();
    notifyPropertyListChanged();
}

int SpringBoneSimulator3D::getSettingCount() const
{
    return static_cast<int>(mSettings.size());
}

void SpringBoneSimulator3D::clearSettings()
{
    setSettingCount(0);
}

int SpringBoneSimulator3D::addSetting(const ct::String& rootBoneName, const ct::String& endBoneName)
{
    const int index = getSettingCount();
    setSettingCount(index + 1);
    setRootBoneName(index, rootBoneName);
    setEndBoneName(index, endBoneName);
    return index;
}

void SpringBoneSimulator3D::setIndividualConfig(int index, bool enabled)
{
    if (!validIndex(index)) return;
    mSettings[index].individualConfig = enabled;
    makeJointsDirty(index, true);
    notifyPropertyListChanged();
}

bool SpringBoneSimulator3D::isConfigIndividual(int index) const
{
    if (!validIndex(index)) return false;
    return mSettings[index].individualConfig;
}

const ct::String& SpringBoneSimulator3D::getJointBoneName(int index, int joint) const
{
    if (!validJoint(index, joint)) return kEmptyString;
    return mSettings[index].joints[joint].boneName;
}

void SpringBoneSimulator3D::setJointBone(int index, int joint, int bone)
{
    if (!validJoint(index, joint)) return;
    JointSetting& setting = mSettings[index].joints[joint];
    setting.bone = bone;
    Skeleton3D* sk = getSkeleton();
    if (sk)
    {
        if (setting.bone <= -1 || static_cast<std::uint32_t>(setting.bone) >= sk->getBoneCount())
        {
            log_warn("spring bone: setting %d: joint %d: bone index '%d' is out of range", index,
                    joint, bone);
            setting.bone = -1;
        }
        else
        {
            setting.boneName = sk->getBoneName(setting.bone);
        }
    }
}

int SpringBoneSimulator3D::getJointBone(int index, int joint) const
{
    if (!validJoint(index, joint)) return -1;
    return mSettings[index].joints[joint].bone;
}

void SpringBoneSimulator3D::setJointRadius(int index, int joint, float radius)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].radius = radius;
}

float SpringBoneSimulator3D::getJointRadius(int index, int joint) const
{
    if (!validJoint(index, joint)) return 0.0f;
    return mSettings[index].joints[joint].radius;
}

void SpringBoneSimulator3D::setJointStiffness(int index, int joint, float stiffness)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].stiffness = stiffness;
}

float SpringBoneSimulator3D::getJointStiffness(int index, int joint) const
{
    if (!validJoint(index, joint)) return 0.0f;
    return mSettings[index].joints[joint].stiffness;
}

void SpringBoneSimulator3D::setJointDrag(int index, int joint, float drag)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].drag = drag;
}

float SpringBoneSimulator3D::getJointDrag(int index, int joint) const
{
    if (!validJoint(index, joint)) return 0.0f;
    return mSettings[index].joints[joint].drag;
}

void SpringBoneSimulator3D::setJointGravity(int index, int joint, float gravity)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].gravity = gravity;
}

float SpringBoneSimulator3D::getJointGravity(int index, int joint) const
{
    if (!validJoint(index, joint)) return 0.0f;
    return mSettings[index].joints[joint].gravity;
}

void SpringBoneSimulator3D::setJointGravityDirection(int index, int joint,
        const Math::Vec3& gravityDirection)
{
    if (!validIndex(index)) return;
    if (modifiermath::isZeroApprox(gravityDirection))
    {
        log_error("spring bone: the gravity direction cannot be zero");
        return;
    }
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].gravityDirection = gravityDirection;
}

Math::Vec3 SpringBoneSimulator3D::getJointGravityDirection(int index, int joint) const
{
    if (!validJoint(index, joint)) return Math::Vec3(0.0f, -1.0f, 0.0f);
    return mSettings[index].joints[joint].gravityDirection;
}

void SpringBoneSimulator3D::setJointRotationAxis(int index, int joint, RotationAxis axis)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    mSettings[index].joints[joint].rotationAxis = axis;
    Skeleton3D* sk = getSkeleton();
    if (sk) validateRotationAxis(sk, index, joint);
    notifyPropertyListChanged();
    mSettings[index].simulationDirty = true;
}

SkeletonModifier3D::RotationAxis SpringBoneSimulator3D::getJointRotationAxis(int index,
        int joint) const
{
    if (!validJoint(index, joint)) return RotationAxis::All;
    return mSettings[index].joints[joint].rotationAxis;
}

void SpringBoneSimulator3D::setJointRotationAxisVector(int index, int joint,
        const Math::Vec3& vector)
{
    if (!validIndex(index)) return;
    if (!isConfigIndividual(index)) return;
    if (!validJoint(index, joint)) return;
    if (mSettings[index].joints[joint].rotationAxis != RotationAxis::Custom) return;
    mSettings[index].joints[joint].rotationAxisVector = vector;
    Skeleton3D* sk = getSkeleton();
    if (sk) validateRotationAxis(sk, index, joint);
    mSettings[index].simulationDirty = true;
}

Math::Vec3 SpringBoneSimulator3D::getJointRotationAxisVector(int index, int joint) const
{
    if (!validJoint(index, joint)) return Math::Vec3(0.0f, 0.0f, 0.0f);
    return mSettings[index].joints[joint].getRotationAxisVector();
}

void SpringBoneSimulator3D::setJointCount(int index, int count)
{
    if (!validIndex(index)) return;
    if (count < 0)
    {
        log_error("spring bone: the joint count cannot be negative");
        return;
    }
    mSettings[index].joints.resize(static_cast<std::size_t>(count));
    notifyPropertyListChanged();
}

int SpringBoneSimulator3D::getJointCount(int index) const
{
    if (!validIndex(index)) return 0;
    return static_cast<int>(mSettings[index].joints.size());
}

void SpringBoneSimulator3D::setEnableAllChildCollisions(int index, bool enabled)
{
    if (!validIndex(index)) return;
    mSettings[index].enableAllChildCollisions = enabled;
    notifyPropertyListChanged();
}

bool SpringBoneSimulator3D::areAllChildCollisionsEnabled(int index) const
{
    if (!validIndex(index)) return false;
    return mSettings[index].enableAllChildCollisions;
}

void SpringBoneSimulator3D::setExcludeCollisionPath(int index, int collision,
        const ct::String& nodePath)
{
    if (!validIndex(index)) return;
    if (!areAllChildCollisionsEnabled(index)) return;
    ct::Vector<ct::String>& list = mSettings[index].excludeCollisions;
    if (collision < 0 || static_cast<std::size_t>(collision) >= list.size())
    {
        log_error("spring bone: exclude collision index %d is out of range", collision);
        return;
    }
    list[collision] = ct::String();
    if (isInsideTree())
    {
        Node* node = nodePath.empty() ? nullptr : getNode(nodePath);
        if (!node)
        {
            makeCollisionsDirty();
            return;
        }
        if (node->getParent() != this)
        {
            makeCollisionsDirty();
            log_error("spring bone: collision must be child of current SpringBoneSimulator3D");
            return;
        }
    }
    list[collision] = nodePath;
    makeCollisionsDirty();
}

const ct::String& SpringBoneSimulator3D::getExcludeCollisionPath(int index, int collision) const
{
    if (!validIndex(index)) return kEmptyString;
    const ct::Vector<ct::String>& list = mSettings[index].excludeCollisions;
    if (collision < 0 || static_cast<std::size_t>(collision) >= list.size())
    {
        log_error("spring bone: exclude collision index %d is out of range", collision);
        return kEmptyString;
    }
    return list[collision];
}

void SpringBoneSimulator3D::setExcludeCollisionCount(int index, int count)
{
    if (!validIndex(index)) return;
    if (!areAllChildCollisionsEnabled(index)) return;
    if (count < 0)
    {
        log_error("spring bone: the exclude collision count cannot be negative");
        return;
    }
    mSettings[index].excludeCollisions.resize(static_cast<std::size_t>(count));
    makeCollisionsDirty();
    notifyPropertyListChanged();
}

int SpringBoneSimulator3D::getExcludeCollisionCount(int index) const
{
    if (!validIndex(index)) return 0;
    return static_cast<int>(mSettings[index].excludeCollisions.size());
}

void SpringBoneSimulator3D::clearExcludeCollisions(int index)
{
    if (!validIndex(index)) return;
    if (!areAllChildCollisionsEnabled(index)) return;
    setExcludeCollisionCount(index, 0);
}

void SpringBoneSimulator3D::setCollisionPath(int index, int collision, const ct::String& nodePath)
{
    if (!validIndex(index)) return;
    if (areAllChildCollisionsEnabled(index)) return;
    ct::Vector<ct::String>& list = mSettings[index].collisions;
    if (collision < 0 || static_cast<std::size_t>(collision) >= list.size())
    {
        log_error("spring bone: collision index %d is out of range", collision);
        return;
    }
    list[collision] = ct::String();
    if (isInsideTree())
    {
        Node* node = nodePath.empty() ? nullptr : getNode(nodePath);
        if (!node)
        {
            makeCollisionsDirty();
            return;
        }
        if (node->getParent() != this)
        {
            makeCollisionsDirty();
            log_error("spring bone: collision must be child of current SpringBoneSimulator3D");
            return;
        }
    }
    list[collision] = nodePath;
    makeCollisionsDirty();
}

const ct::String& SpringBoneSimulator3D::getCollisionPath(int index, int collision) const
{
    if (!validIndex(index)) return kEmptyString;
    const ct::Vector<ct::String>& list = mSettings[index].collisions;
    if (collision < 0 || static_cast<std::size_t>(collision) >= list.size())
    {
        log_error("spring bone: collision index %d is out of range", collision);
        return kEmptyString;
    }
    return list[collision];
}

void SpringBoneSimulator3D::setCollisionCount(int index, int count)
{
    if (!validIndex(index)) return;
    if (areAllChildCollisionsEnabled(index)) return;
    if (count < 0)
    {
        log_error("spring bone: the collision count cannot be negative");
        return;
    }
    mSettings[index].collisions.resize(static_cast<std::size_t>(count));
    makeCollisionsDirty();
    notifyPropertyListChanged();
}

int SpringBoneSimulator3D::getCollisionCount(int index) const
{
    if (!validIndex(index)) return 0;
    return static_cast<int>(mSettings[index].collisions.size());
}

void SpringBoneSimulator3D::clearCollisions(int index)
{
    if (!validIndex(index)) return;
    if (areAllChildCollisionsEnabled(index)) return;
    setCollisionCount(index, 0);
}

ct::Vector<SpringBoneCollision3D*> SpringBoneSimulator3D::getValidCollisions(int index)
{
    if (!validIndex(index)) return ct::Vector<SpringBoneCollision3D*>();
    if (mCollisionsDirty) findCollisions();
    return mSettings[index].cachedCollisions;
}

void SpringBoneSimulator3D::setExternalForce(const Math::Vec3& force)
{
    mExternalForce = force;
}

Math::Vec3 SpringBoneSimulator3D::getExternalForce() const
{
    return mExternalForce;
}

void SpringBoneSimulator3D::setMutableBoneAxes(bool enabled)
{
    mMutableBoneAxes = enabled;
    for (std::size_t i = 0; i < mSettings.size(); ++i) mSettings[i].simulationDirty = true;
}

bool SpringBoneSimulator3D::areBoneAxesMutable() const
{
    return mMutableBoneAxes;
}

void SpringBoneSimulator3D::skeletonChanged(Skeleton3D*, Skeleton3D*)
{
    makeAllJointsDirty();
}

void SpringBoneSimulator3D::validateBoneNames()
{
    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        const int index = static_cast<int>(i);
        if (!mSettings[i].rootBoneName.empty()) setRootBoneName(index, mSettings[i].rootBoneName);
        else if (mSettings[i].rootBone != -1)
            setRootBone(index, mSettings[i].rootBone);
        if (!mSettings[i].endBoneName.empty()) setEndBoneName(index, mSettings[i].endBoneName);
        else if (mSettings[i].endBone != -1)
            setEndBone(index, mSettings[i].endBone);
    }
}

void SpringBoneSimulator3D::restUpdated()
{
    validateBoneNames();
    makeAllJointsDirty();
}

void SpringBoneSimulator3D::makeJointsDirty(int index, bool reset)
{
    if (!validIndex(index)) return;
    mSettings[index].jointsDirty = true;
    mJointsReset = mJointsReset || reset;
    mJointsDirty = true;
}

void SpringBoneSimulator3D::makeAllJointsDirty()
{
    for (std::size_t i = 0; i < mSettings.size(); ++i) updateJointArray(static_cast<int>(i));
}

void SpringBoneSimulator3D::addChildNotify(Node* child)
{
    SkeletonModifier3D::addChildNotify(child);
    makeCollisionsDirty();
}

void SpringBoneSimulator3D::moveChildNotify(Node* child)
{
    SkeletonModifier3D::moveChildNotify(child);
    makeCollisionsDirty();
}

void SpringBoneSimulator3D::removeChildNotify(Node* child)
{
    SkeletonModifier3D::removeChildNotify(child);
    makeCollisionsDirty();
}

void SpringBoneSimulator3D::validateRotationAxes(Skeleton3D* skeleton) const
{
    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        for (std::size_t j = 0; j < mSettings[i].joints.size(); ++j)
            validateRotationAxis(skeleton, static_cast<int>(i), static_cast<int>(j));
    }
}

void SpringBoneSimulator3D::validateRotationAxis(Skeleton3D* skeleton, int index, int joint) const
{
    const Setting& setting = mSettings[index];
    const RotationAxis axis = setting.joints[joint].rotationAxis;
    if (axis == RotationAxis::All) return;
    Math::Vec3 rot = normalized(getJointRotationAxisVector(index, joint));
    Math::Vec3 fwd;
    if (joint < static_cast<int>(setting.joints.size()) - 1)
    {
        fwd = skeleton->getBoneRest(setting.joints[joint + 1].bone).GetTranslation();
    }
    else if (setting.extendEndBone)
    {
        fwd = getEndBoneAxis(setting.endBone, setting.endBoneDirection);
        if (modifiermath::isZeroApprox(fwd)) return;
    }
    fwd = normalized(fwd);
    if (modifiermath::isEqualApprox(std::fabs(rot.Dot(fwd)), 1.0f))
    {
        log_warn("spring bone: setting %d joint %d: rotation axis and forward vector are "
                 "colinear, this is not advised as it may cause unwanted rotation",
                index, joint);
    }
}

void SpringBoneSimulator3D::findCollisions()
{
    if (!mCollisionsDirty) return;
    mCollisions.clear();
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        if (SpringBoneCollision3D* collision = getChild(i)->castTo<SpringBoneCollision3D>())
            mCollisions.push_back(collision);
    }

    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        Setting& setting = mSettings[i];
        ct::Vector<SpringBoneCollision3D*>& cache = setting.cachedCollisions;
        cache.clear();
        if (!setting.enableAllChildCollisions)
        {
            for (std::size_t j = 0; j < setting.collisions.size(); ++j)
            {
                Node* node =
                        setting.collisions[j].empty() ? nullptr : getNode(setting.collisions[j]);
                if (!node) continue;
                SpringBoneCollision3D* collision = node->castTo<SpringBoneCollision3D>();
                bool found = false;
                for (std::size_t k = 0; collision && k < mCollisions.size(); ++k)
                    found = found || mCollisions[k] == collision;
                if (!found) setting.collisions[j] = ct::String();
                else
                    cache.push_back(collision);
            }
        }
        else
        {
            ct::Vector<std::size_t> masks;
            for (std::size_t j = 0; j < setting.excludeCollisions.size(); ++j)
            {
                Node* node = setting.excludeCollisions[j].empty()
                                     ? nullptr
                                     : getNode(setting.excludeCollisions[j]);
                if (!node) continue;
                SpringBoneCollision3D* collision = node->castTo<SpringBoneCollision3D>();
                std::size_t found = mCollisions.size();
                for (std::size_t k = 0; collision && k < mCollisions.size(); ++k)
                {
                    if (mCollisions[k] == collision)
                    {
                        found = k;
                        break;
                    }
                }
                if (found == mCollisions.size())
                {
                    setting.excludeCollisions[j] = ct::String();
                }
                else
                {
                    std::size_t at = masks.size();
                    masks.push_back(found);
                    while (at > 0 && masks[at - 1] > masks[at])
                    {
                        const std::size_t swap = masks[at - 1];
                        masks[at - 1] = masks[at];
                        masks[at] = swap;
                        --at;
                    }
                }
            }
            std::size_t maskIndex = 0;
            for (std::size_t j = 0; j < mCollisions.size(); ++j)
            {
                if (maskIndex < masks.size() && j == masks[maskIndex])
                {
                    ++maskIndex;
                    continue;
                }
                cache.push_back(mCollisions[j]);
            }
        }
    }

    mCollisionsDirty = false;
}

void SpringBoneSimulator3D::processCollisions()
{
    for (std::size_t i = 0; i < mCollisions.size(); ++i)
    {
        if (mCollisions[i]) mCollisions[i]->syncPose();
    }
}

void SpringBoneSimulator3D::makeCollisionsDirty()
{
    mCollisionsDirty = true;
}

void SpringBoneSimulator3D::updateJointArray(int index)
{
    makeJointsDirty(index, true);

    Skeleton3D* sk = getSkeleton();
    int currentBone = mSettings[index].endBone;
    const int rootBone = mSettings[index].rootBone;
    if (!sk || currentBone < 0 || rootBone < 0)
    {
        setJointCount(index, 0);
        return;
    }

    bool valid = false;
    while (currentBone >= 0)
    {
        if (currentBone == rootBone)
        {
            valid = true;
            break;
        }
        currentBone = sk->getBoneParent(currentBone);
    }

    if (!valid)
    {
        setJointCount(index, 0);
        log_error("spring bone: end bone must be the same as or a child of the root bone");
        return;
    }

    ct::Vector<int> newJoints;
    currentBone = mSettings[index].endBone;
    while (currentBone != rootBone)
    {
        newJoints.push_back(currentBone);
        currentBone = sk->getBoneParent(currentBone);
    }
    newJoints.push_back(currentBone);
    for (std::size_t i = 0, j = newJoints.size(); i + 1 < j; ++i, --j)
    {
        const int swap = newJoints[i];
        newJoints[i] = newJoints[j - 1];
        newJoints[j - 1] = swap;
    }

    setJointCount(index, static_cast<int>(newJoints.size()));
    for (std::size_t i = 0; i < newJoints.size(); ++i)
        setJointBone(index, static_cast<int>(i), newJoints[i]);
}

void SpringBoneSimulator3D::updateJoints()
{
    if (!mJointsDirty) return;
    const bool reset = mJointsReset;
    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        Setting& setting = mSettings[i];
        if (!setting.jointsDirty) continue;
        if (setting.individualConfig)
        {
            setting.simulationDirty = reset;
            setting.jointsDirty = false;
            continue;
        }
        ct::Vector<JointSetting>& joints = setting.joints;
        const float unit =
                joints.size() > 1 ? (1.0f / static_cast<float>(joints.size() - 1)) : 0.0f;
        for (std::size_t j = 0; j < joints.size(); ++j)
        {
            const float offset = static_cast<float>(j) * unit;

            if (setting.radiusDampingCurve.valid())
                joints[j].radius = setting.radius * setting.radiusDampingCurve.sampleBaked(offset);
            else
                joints[j].radius = setting.radius;

            if (setting.stiffnessDampingCurve.valid())
                joints[j].stiffness =
                        setting.stiffness * setting.stiffnessDampingCurve.sampleBaked(offset);
            else
                joints[j].stiffness = setting.stiffness;

            if (setting.dragDampingCurve.valid())
                joints[j].drag = setting.drag * setting.dragDampingCurve.sampleBaked(offset);
            else
                joints[j].drag = setting.drag;

            if (setting.gravityDampingCurve.valid())
                joints[j].gravity =
                        setting.gravity * setting.gravityDampingCurve.sampleBaked(offset);
            else
                joints[j].gravity = setting.gravity;

            joints[j].gravityDirection = setting.gravityDirection;
            joints[j].rotationAxis = setting.rotationAxis;
            joints[j].rotationAxisVector = setting.rotationAxisVector;
        }
        setting.simulationDirty = reset;
        setting.jointsDirty = false;
    }
    mJointsDirty = false;
    mJointsReset = false;
    Skeleton3D* sk = getSkeleton();
    if (sk) validateRotationAxes(sk);
}

void SpringBoneSimulator3D::updateBoneAxis(Skeleton3D* skeleton, Setting& setting)
{
    const ct::Vector<JointSetting>& joints = setting.joints;
    const int len = static_cast<int>(joints.size()) - 1;
    for (int j = 0; j < len; ++j)
    {
        if (!joints[j].hasVerlet) continue;
        const Math::Vec3 axis = skeleton->getBonePose(joints[j + 1].bone).position;
        if (modifiermath::isZeroApprox(axis)) continue;
        setting.joints[j].verlet.forwardVector =
                snapVectorToPlane(joints[j].getRotationAxisVector(), normalized(axis));
        setting.joints[j].verlet.length = axis.Length();
    }
    if (setting.extendEndBone && len >= 0)
    {
        if (joints[len].hasVerlet)
        {
            const Math::Vec3 axis = getEndBoneAxis(setting.endBone, setting.endBoneDirection);
            if (!modifiermath::isZeroApprox(axis))
            {
                setting.joints[len].verlet.forwardVector =
                        snapVectorToPlane(joints[len].getRotationAxisVector(), normalized(axis));
                setting.joints[len].verlet.length = setting.endBoneLength;
            }
        }
    }
}

Math::Vec3 SpringBoneSimulator3D::getBoneVector(int index, int joint) const
{
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton) return Math::Vec3(0.0f, 0.0f, 0.0f);
    if (!validJoint(index, joint)) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const Setting& setting = mSettings[index];
    const ct::Vector<JointSetting>& joints = setting.joints;
    if (!joints[joint].hasVerlet)
    {
        if (joint == static_cast<int>(joints.size()) - 1)
            return getEndBoneAxis(setting.endBone, setting.endBoneDirection) *
                   setting.endBoneLength;
        return mMutableBoneAxes ? skeleton->getBonePose(joints[joint + 1].bone).position
                                : skeleton->getBoneRest(joints[joint + 1].bone).GetTranslation();
    }
    return joints[joint].verlet.forwardVector * joints[joint].verlet.length;
}

void SpringBoneSimulator3D::activeChanged(bool active)
{
    if (active) reset();
}

void SpringBoneSimulator3D::processModificationImpl(double delta)
{
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton) return;
    updateJoints();
    findCollisions();
    processCollisions();

    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        initJoints(skeleton, mSettings[i]);
        Setting& setting = mSettings[i];
        processJoints(delta, skeleton, setting.joints, setting.cachedCollisions,
                setting.cachedCenter, setting.cachedInvertedCenter,
                modifiermath::rotationQuaternion(setting.cachedInvertedCenter));
    }
}

void SpringBoneSimulator3D::reset()
{
    if (!isInsideTree()) return;
    Skeleton3D* skeleton = getSkeleton();
    if (!skeleton) return;
    findCollisions();
    processCollisions();
    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        makeJointsDirty(static_cast<int>(i), true);
        initJoints(skeleton, mSettings[i]);
    }
}

void SpringBoneSimulator3D::initJoints(Skeleton3D* skeleton, Setting& setting)
{
    if (setting.centerFrom == CenterFrom::WorldOrigin)
    {
        setting.cachedCenter = skeleton->getGlobalTransform();
    }
    else if (setting.centerFrom == CenterFrom::Node)
    {
        if (setting.centerNode.empty())
        {
            setting.cachedCenter = Math::Mat4::Identity();
        }
        else
        {
            Node* node = getNode(setting.centerNode);
            Node3D* node3d = node ? node->castTo<Node3D>() : nullptr;
            if (!node3d) setting.cachedCenter = Math::Mat4::Identity();
            else
                setting.cachedCenter = node3d->getGlobalTransform().InverseAffine() *
                                       skeleton->getGlobalTransform();
        }
    }
    else
    {
        if (setting.centerBone >= 0)
            setting.cachedCenter = skeleton->getBoneGlobalPose(setting.centerBone);
        else
            setting.cachedCenter = Math::Mat4::Identity();
    }
    setting.cachedInvertedCenter = setting.cachedCenter.InverseAffine();

    if (!setting.simulationDirty)
    {
        if (mMutableBoneAxes) updateBoneAxis(skeleton, setting);
        return;
    }
    for (std::size_t i = 0; i < setting.joints.size(); ++i)
    {
        JointSetting& joint = setting.joints[i];
        joint.hasVerlet = false;
        if (i + 1 < setting.joints.size())
        {
            const Math::Vec3 axis =
                    skeleton->getBoneRest(setting.joints[i + 1].bone).GetTranslation();
            if (modifiermath::isZeroApprox(axis)) continue;
            joint.hasVerlet = true;
            joint.verlet = VerletInfo();
            joint.verlet.currentTail = transformPoint(setting.cachedCenter,
                    transformPoint(skeleton->getBoneGlobalPose(joint.bone), axis));
            joint.verlet.prevTail = joint.verlet.currentTail;
            joint.verlet.forwardVector =
                    snapVectorToPlane(joint.getRotationAxisVector(), normalized(axis));
            joint.verlet.length = axis.Length();
            joint.verlet.currentRot = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        }
        else if (setting.extendEndBone && setting.endBoneLength > 0.0f)
        {
            const Math::Vec3 axis = getEndBoneAxis(setting.endBone, setting.endBoneDirection);
            if (modifiermath::isZeroApprox(axis)) continue;
            joint.hasVerlet = true;
            joint.verlet = VerletInfo();
            joint.verlet.forwardVector =
                    snapVectorToPlane(joint.getRotationAxisVector(), normalized(axis));
            joint.verlet.length = setting.endBoneLength;
            joint.verlet.currentTail = transformPoint(setting.cachedCenter,
                    transformPoint(skeleton->getBoneGlobalPose(joint.bone),
                            axis * setting.endBoneLength));
            joint.verlet.prevTail = joint.verlet.currentTail;
            joint.verlet.currentRot = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        }
    }
    if (mMutableBoneAxes) updateBoneAxis(skeleton, setting);
    setting.simulationDirty = false;
}

void SpringBoneSimulator3D::processJoints(double delta, Skeleton3D* skeleton,
        ct::Vector<JointSetting>& joints, const ct::Vector<SpringBoneCollision3D*>& collisions,
        const Math::Mat4& centerTransform, const Math::Mat4& invertedCenterTransform,
        const Math::Quaternion& invertedCenterRotation)
{
    const float step = static_cast<float>(delta);
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        if (!joints[i].hasVerlet) continue;
        VerletInfo& verlet = joints[i].verlet;
        const Math::Mat4 currentGlobalPose = skeleton->getBoneGlobalPose(joints[i].bone);
        const Math::Mat4 currentWorldPose = centerTransform * currentGlobalPose;
        const Math::Quaternion currentRot = modifiermath::rotationQuaternion(currentGlobalPose);
        const Math::Vec3 currentOrigin =
                transformPoint(centerTransform, currentGlobalPose.GetTranslation());
        const Math::Vec3 external = rotateBy(invertedCenterRotation,
                (mExternalForce + joints[i].gravityDirection * joints[i].gravity) * step);

        Math::Vec3 nextTail =
                verlet.currentTail +
                (verlet.currentTail - verlet.prevTail) * (1.0f - joints[i].drag) +
                rotateBy(modifiermath::rotationQuaternion(centerTransform),
                        rotateBy(currentRot, verlet.forwardVector * (joints[i].stiffness * step)) +
                                external);
        const Math::Vec3 worldOrigin = currentWorldPose.GetTranslation();
        const Math::Quaternion worldRotation = modifiermath::rotationQuaternion(currentWorldPose);
        if (joints[i].rotationAxis != RotationAxis::All)
        {
            nextTail = worldOrigin +
                       rotateBy(worldRotation, snapVectorToPlane(joints[i].getRotationAxisVector(),
                                                       modifiermath::xformInverse(worldRotation,
                                                               nextTail - worldOrigin)));
        }
        nextTail = limitLength(currentOrigin, nextTail, verlet.length);

        for (std::size_t j = 0; j < collisions.size(); ++j)
        {
            SpringBoneCollision3D* collision = collisions[j];
            if (!collision) continue;
            nextTail =
                    collision->collide(centerTransform, joints[i].radius, verlet.length, nextTail);
            if (joints[i].rotationAxis != RotationAxis::All)
            {
                nextTail =
                        worldOrigin +
                        rotateBy(worldRotation, snapVectorToPlane(joints[i].getRotationAxisVector(),
                                                        modifiermath::xformInverse(worldRotation,
                                                                nextTail - worldOrigin)));
            }
            nextTail = limitLength(currentOrigin, nextTail, verlet.length);
        }

        verlet.prevTail = verlet.currentTail;
        verlet.currentTail = nextTail;

        Math::Vec3 from = rotateBy(currentRot, verlet.forwardVector);
        Math::Vec3 to = invertedCenterTransform.UpperLeft3x3() * (nextTail - currentOrigin);
        from = normalized(from);
        to = normalized(to);
        Math::Quaternion fromTo = getFromToRotation(from, to, verlet.currentRot);
        verlet.currentRot = fromTo;

        fromTo = fromTo * currentRot;
        fromTo = getLocalPoseRotation(skeleton, joints[i].bone, fromTo);
        skeleton->setBonePoseRotation(joints[i].bone, fromTo);
    }
}

namespace
{

ct::Json curveToJson(const ct::Vector<float>& samples)
{
    return scenejson::fromFloats(samples.data(), samples.size());
}

void readCurve(const ct::Json& entry, const char* key, ct::Vector<float>& out, SceneResult& result,
        const ct::String& path)
{
    out.clear();
    const ct::Json* found = entry.find(key);
    if (!found) return;
    if (!scenejson::toFloatVector(*found, out))
    {
        out.clear();
        result.addError(scenejson::field(path, key), "expected an array of numbers");
    }
}

bool readText(const ct::Json& entry, const char* key, ct::String& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = entry.find(key);
    if (!found) return false;
    if (!found->is_string())
    {
        result.addError(scenejson::field(path, key), "expected a text");
        return false;
    }
    out = found->str();
    return true;
}

void readStringList(const ct::Json& entry, const char* key, ct::Vector<ct::String>& out,
        SceneResult& result, const ct::String& path)
{
    out.clear();
    const ct::Json* found = entry.find(key);
    if (!found) return;
    if (!found->is_array())
    {
        result.addError(scenejson::field(path, key), "expected an array of node paths");
        return;
    }
    for (std::size_t i = 0; i < found->size(); ++i)
    {
        if (!found->at(i).is_string())
        {
            result.addError(scenejson::element(path, key, i), "expected a node path");
            out.push_back(ct::String());
            continue;
        }
        out.push_back(found->at(i).str());
    }
}

} // namespace

bool SpringBoneSimulator3D::saveData(ct::Json& data, SceneResult& result,
        const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mSettings.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kSpringBoneDataVersion));
    ct::Json settings = ct::Json::array();
    for (std::size_t i = 0; i < mSettings.size(); ++i)
    {
        const Setting& setting = mSettings[i];
        ct::Json entry = ct::Json::object();
        if (!setting.rootBoneName.empty()) entry["root_bone_name"] = ct::Json(setting.rootBoneName);
        else if (setting.rootBone != -1)
            entry["root_bone"] = ct::Json(static_cast<long long>(setting.rootBone));
        if (!setting.endBoneName.empty()) entry["end_bone_name"] = ct::Json(setting.endBoneName);
        else if (setting.endBone != -1)
            entry["end_bone"] = ct::Json(static_cast<long long>(setting.endBone));
        entry["extend_end_bone"] = ct::Json(setting.extendEndBone);
        entry["end_bone_direction"] =
                ct::Json(static_cast<long long>(static_cast<int>(setting.endBoneDirection)));
        entry["end_bone_length"] = scenejson::fromFloat(setting.endBoneLength);
        entry["center_from"] =
                ct::Json(static_cast<long long>(static_cast<int>(setting.centerFrom)));
        if (!setting.centerNode.empty()) entry["center_node"] = ct::Json(setting.centerNode);
        if (!setting.centerBoneName.empty())
            entry["center_bone_name"] = ct::Json(setting.centerBoneName);
        else if (setting.centerBone != -1)
            entry["center_bone"] = ct::Json(static_cast<long long>(setting.centerBone));
        entry["individual_config"] = ct::Json(setting.individualConfig);
        entry["rotation_axis"] =
                ct::Json(static_cast<long long>(static_cast<int>(setting.rotationAxis)));
        entry["rotation_axis_vector"] = scenejson::fromVec3(setting.rotationAxisVector);
        entry["radius"] = scenejson::fromFloat(setting.radius);
        if (setting.radiusDampingCurve.valid())
            entry["radius_damping_curve"] = curveToJson(setting.radiusDampingCurve.samples);
        entry["stiffness"] = scenejson::fromFloat(setting.stiffness);
        if (setting.stiffnessDampingCurve.valid())
            entry["stiffness_damping_curve"] = curveToJson(setting.stiffnessDampingCurve.samples);
        entry["drag"] = scenejson::fromFloat(setting.drag);
        if (setting.dragDampingCurve.valid())
            entry["drag_damping_curve"] = curveToJson(setting.dragDampingCurve.samples);
        entry["gravity"] = scenejson::fromFloat(setting.gravity);
        if (setting.gravityDampingCurve.valid())
            entry["gravity_damping_curve"] = curveToJson(setting.gravityDampingCurve.samples);
        entry["gravity_direction"] = scenejson::fromVec3(setting.gravityDirection);

        if (setting.individualConfig)
        {
            ct::Json joints = ct::Json::array();
            for (std::size_t j = 0; j < setting.joints.size(); ++j)
            {
                const JointSetting& joint = setting.joints[j];
                ct::Json item = ct::Json::object();
                item["rotation_axis"] =
                        ct::Json(static_cast<long long>(static_cast<int>(joint.rotationAxis)));
                item["rotation_axis_vector"] = scenejson::fromVec3(joint.rotationAxisVector);
                item["radius"] = scenejson::fromFloat(joint.radius);
                item["stiffness"] = scenejson::fromFloat(joint.stiffness);
                item["drag"] = scenejson::fromFloat(joint.drag);
                item["gravity"] = scenejson::fromFloat(joint.gravity);
                item["gravity_direction"] = scenejson::fromVec3(joint.gravityDirection);
                joints.push_back(item);
            }
            entry["joints"] = joints;
        }

        entry["enable_all_child_collisions"] = ct::Json(setting.enableAllChildCollisions);
        const ct::Vector<ct::String>& list =
                setting.enableAllChildCollisions ? setting.excludeCollisions : setting.collisions;
        ct::Json paths = ct::Json::array();
        for (std::size_t j = 0; j < list.size(); ++j) paths.push_back(ct::Json(list[j]));
        if (paths.size() > 0)
            entry[setting.enableAllChildCollisions ? "exclude_collisions" : "collisions"] = paths;

        settings.push_back(entry);
    }
    data["settings"] = settings;
    return true;
}

bool SpringBoneSimulator3D::loadData(const ct::Json& data, SceneResult& result,
        const ct::String& path)
{
    if (!scenejson::checkVersion(data, kSpringBoneDataVersion, result, path)) return false;

    clearSettings();
    const ct::Json* settings = scenejson::arrayField(data, "settings", result, path);
    if (!settings) return true;

    setSettingCount(static_cast<int>(settings->size()));
    for (std::size_t i = 0; i < settings->size(); ++i)
    {
        const ct::Json& entry = settings->at(i);
        const ct::String entryPath = scenejson::element(path, "settings", i);
        if (!entry.is_object())
        {
            result.addError(entryPath, "expected an object");
            continue;
        }
        const int index = static_cast<int>(i);

        ct::String text;
        std::int64_t integer = 0;
        float number = 0.0f;
        bool flag = false;
        Math::Vec3 vector;

        if (readText(entry, "root_bone_name", text, result, entryPath))
            setRootBoneName(index, text);
        if (scenejson::readIntField(entry, "root_bone", integer, result, entryPath))
            setRootBone(index, static_cast<int>(integer));
        if (readText(entry, "end_bone_name", text, result, entryPath)) setEndBoneName(index, text);
        if (scenejson::readIntField(entry, "end_bone", integer, result, entryPath))
            setEndBone(index, static_cast<int>(integer));
        if (scenejson::readBoolField(entry, "extend_end_bone", flag, result, entryPath))
            setExtendEndBone(index, flag);
        if (scenejson::readIntField(entry, "end_bone_direction", integer, result, entryPath))
        {
            if (integer < 0 || integer > static_cast<std::int64_t>(BoneDirection::FromParent))
                result.addError(scenejson::field(entryPath, "end_bone_direction"),
                        "not a valid direction");
            else
                setEndBoneDirection(index, static_cast<BoneDirection>(integer));
        }
        if (scenejson::readFloatField(entry, "end_bone_length", number, result, entryPath))
            setEndBoneLength(index, number);
        if (scenejson::readIntField(entry, "center_from", integer, result, entryPath))
        {
            if (integer < 0 || integer > static_cast<std::int64_t>(CenterFrom::Bone))
                result.addError(scenejson::field(entryPath, "center_from"), "not a valid center");
            else
                setCenterFrom(index, static_cast<CenterFrom>(integer));
        }
        if (readText(entry, "center_node", text, result, entryPath)) setCenterNode(index, text);
        if (readText(entry, "center_bone_name", text, result, entryPath))
            setCenterBoneName(index, text);
        if (scenejson::readIntField(entry, "center_bone", integer, result, entryPath))
            setCenterBone(index, static_cast<int>(integer));

        if (scenejson::readIntField(entry, "rotation_axis", integer, result, entryPath))
        {
            if (integer < 0 || integer > static_cast<std::int64_t>(RotationAxis::Custom))
                result.addError(scenejson::field(entryPath, "rotation_axis"),
                        "not a valid rotation axis");
            else
                setRotationAxis(index, static_cast<RotationAxis>(integer));
        }
        if (scenejson::readVec3Field(entry, "rotation_axis_vector", vector, result, entryPath))
            setRotationAxisVector(index, vector);
        ct::Vector<float> curve;
        if (scenejson::readFloatField(entry, "radius", number, result, entryPath))
            setRadius(index, number);
        readCurve(entry, "radius_damping_curve", curve, result, entryPath);
        if (curve.size() > 0) setRadiusDampingCurve(index, curve);
        if (scenejson::readFloatField(entry, "stiffness", number, result, entryPath))
            setStiffness(index, number);
        readCurve(entry, "stiffness_damping_curve", curve, result, entryPath);
        if (curve.size() > 0) setStiffnessDampingCurve(index, curve);
        if (scenejson::readFloatField(entry, "drag", number, result, entryPath))
            setDrag(index, number);
        readCurve(entry, "drag_damping_curve", curve, result, entryPath);
        if (curve.size() > 0) setDragDampingCurve(index, curve);
        if (scenejson::readFloatField(entry, "gravity", number, result, entryPath))
            setGravity(index, number);
        readCurve(entry, "gravity_damping_curve", curve, result, entryPath);
        if (curve.size() > 0) setGravityDampingCurve(index, curve);
        if (scenejson::readVec3Field(entry, "gravity_direction", vector, result, entryPath))
            setGravityDirection(index, vector);

        if (scenejson::readBoolField(entry, "individual_config", flag, result, entryPath))
            setIndividualConfig(index, flag);
        if (const ct::Json* joints = scenejson::arrayField(entry, "joints", result, entryPath))
        {
            setJointCount(index, static_cast<int>(joints->size()));
            for (std::size_t j = 0; j < joints->size(); ++j)
            {
                const ct::Json& item = joints->at(j);
                const ct::String jointPath = scenejson::element(entryPath, "joints", j);
                if (!item.is_object())
                {
                    result.addError(jointPath, "expected an object");
                    continue;
                }
                const int joint = static_cast<int>(j);
                if (scenejson::readIntField(item, "rotation_axis", integer, result, jointPath))
                {
                    if (integer < 0 || integer > static_cast<std::int64_t>(RotationAxis::Custom))
                        result.addError(scenejson::field(jointPath, "rotation_axis"),
                                "not a valid rotation axis");
                    else
                        setJointRotationAxis(index, joint, static_cast<RotationAxis>(integer));
                }
                if (scenejson::readVec3Field(item, "rotation_axis_vector", vector, result,
                            jointPath))
                    setJointRotationAxisVector(index, joint, vector);
                if (scenejson::readFloatField(item, "radius", number, result, jointPath))
                    setJointRadius(index, joint, number);
                if (scenejson::readFloatField(item, "stiffness", number, result, jointPath))
                    setJointStiffness(index, joint, number);
                if (scenejson::readFloatField(item, "drag", number, result, jointPath))
                    setJointDrag(index, joint, number);
                if (scenejson::readFloatField(item, "gravity", number, result, jointPath))
                    setJointGravity(index, joint, number);
                if (scenejson::readVec3Field(item, "gravity_direction", vector, result, jointPath))
                    setJointGravityDirection(index, joint, vector);
            }
        }

        if (scenejson::readBoolField(entry, "enable_all_child_collisions", flag, result, entryPath))
            setEnableAllChildCollisions(index, flag);
        ct::Vector<ct::String> paths;
        if (mSettings[index].enableAllChildCollisions)
        {
            readStringList(entry, "exclude_collisions", paths, result, entryPath);
            setExcludeCollisionCount(index, static_cast<int>(paths.size()));
            for (std::size_t j = 0; j < paths.size(); ++j)
                setExcludeCollisionPath(index, static_cast<int>(j), paths[j]);
        }
        else
        {
            readStringList(entry, "collisions", paths, result, entryPath);
            setCollisionCount(index, static_cast<int>(paths.size()));
            for (std::size_t j = 0; j < paths.size(); ++j)
                setCollisionPath(index, static_cast<int>(j), paths[j]);
        }
    }
    return true;
}

void SpringBoneSimulator3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SpringBoneSimulator3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "external_force"), &P::setExternalForce,
            &P::getExternalForce);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "mutable_bone_axes"),
            &P::setMutableBoneAxes, &P::areBoneAxesMutable);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "setting_count", 0.0, 4096.0, 1.0,
                               true, false, PropertyUsage::Editor),
            &P::setSettingCount, &P::getSettingCount);
}

} // namespace prisma3d
