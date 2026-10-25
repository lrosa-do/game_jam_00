#include "prisma3d/scene/Ragdoll3D.h"

#include "prisma3d/main/Engine.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <ct/vector.hpp>

#include <ctype.h>
#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

constexpr std::size_t kSlots = static_cast<std::size_t>(RagdollBone::Count);
constexpr std::size_t kParts = static_cast<std::size_t>(PhysicsRagdollPart::Count);

struct SlotNames
{
    const char* names[8];
};

const SlotNames kCandidates[kSlots] = { { { "hips", "pelvis", "waist", "root", "hip", nullptr } },
    { { "spine", "spine1", "spine2", "chest", "stomach", "abdomen", nullptr } },
    { { "neck", "neck1", nullptr } }, { { "head", nullptr } },
    { { "leftarm", "upperarm.l", "upperarm_l", "humerus.l", "l_upperarm", "arm.l", "shoulder.l",
        nullptr } },
    { { "leftforearm", "lowerarm.l", "lowerarm_l", "forearm.l", "ulna.l", "l_forearm", "forearm_l",
        nullptr } },
    { { "lefthand", "hand.l", "hand_l", "l_hand", nullptr } },
    { { "rightarm", "upperarm.r", "upperarm_r", "humerus.r", "r_upperarm", "arm.r", "shoulder.r",
        nullptr } },
    { { "rightforearm", "lowerarm.r", "lowerarm_r", "forearm.r", "ulna.r", "r_forearm", "forearm_r",
        nullptr } },
    { { "righthand", "hand.r", "hand_r", "r_hand", nullptr } },
    { { "leftupleg", "thigh.l", "thigh_l", "upleg.l", "l_thigh", "upperleg.l", "leftthigh",
        nullptr } },
    { { "leftleg", "calf.l", "calf_l", "shin.l", "l_calf", "lowerleg.l", "leftcalf", nullptr } },
    { { "leftfoot", "foot.l", "foot_l", "l_foot", nullptr } },
    { { "lefttoebase", "lefttoes", "lefttoe", "toe.l", "toe_l", "toes.l", "l_toe", nullptr } },
    { { "rightupleg", "thigh.r", "thigh_r", "upleg.r", "r_thigh", "upperleg.r", "rightthigh",
        nullptr } },
    { { "rightleg", "calf.r", "calf_r", "shin.r", "r_calf", "lowerleg.r", "rightcalf", nullptr } },
    { { "rightfoot", "foot.r", "foot_r", "r_foot", nullptr } },
    { { "righttoebase", "righttoes", "righttoe", "toe.r", "toe_r", "toes.r", "r_toe", nullptr } } };

ct::String normalized(const ct::String& name)
{
    ct::String out;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        if (c == ' ') continue;
        out += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
    const std::size_t colon = out.find(':');
    if (colon != ct::String::npos) out = out.substr(colon + 1);
    return out;
}

float uniformScale(const Math::Mat4& transform)
{
    return Math::Vec3(transform.col0.x, transform.col0.y, transform.col0.z).Length();
}

} // namespace

Ragdoll3D::Ragdoll3D()
        : Node3D(),
          mSkeleton(nullptr),
          mPlayer(nullptr),
          mRagdoll(),
          mBlend(1.0f),
          mScale(0.0f),
          mFatness(1.0f),
          mHeadSize(1.0f),
          mMask(0xFFFFFFFFu),
          mActive(false),
          mPlayerWasActive(false)
{
    for (std::size_t i = 0; i < kSlots; ++i) mBones[i] = -1;
    setProcess(true);
    setPhysicsProcess(true);
}

Ragdoll3D::~Ragdoll3D()
{
    release();
}

void Ragdoll3D::release()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mRagdoll.valid()) server->ragdollFree(mRagdoll);
    mRagdoll = PhysicsRagdollId();
    mActive = false;
}

void Ragdoll3D::setSkeleton(Skeleton3D* skeleton)
{
    mSkeletonPath = ct::String();
    if (mSkeleton == skeleton) return;
    stop();
    release();
    mSkeleton = skeleton;
}

Skeleton3D* Ragdoll3D::getSkeleton() const
{
    return mSkeleton;
}

void Ragdoll3D::setAnimationPlayer(AnimationPlayer* player)
{
    mPlayerPath = ct::String();
    mPlayer = player;
}

void Ragdoll3D::setSkeletonPath(const ct::String& path)
{
    mSkeletonPath = path;
    resolveSkeleton();
}

ct::String Ragdoll3D::getSkeletonPath() const
{
    if (mSkeleton) return getPathTo(mSkeleton);
    return mSkeletonPath;
}

void Ragdoll3D::resolveSkeleton()
{
    Skeleton3D* skeleton = nullptr;
    if (!mSkeletonPath.empty())
    {
        Node* node = getNode(mSkeletonPath);
        skeleton = node ? node->castTo<Skeleton3D>() : nullptr;
    }
    if (skeleton)
    {
        setSkeleton(skeleton);
        return;
    }
    const ct::String pending = mSkeletonPath;
    setSkeleton(nullptr);
    mSkeletonPath = pending;
}

void Ragdoll3D::setAnimationPlayerPath(const ct::String& path)
{
    mPlayerPath = path;
    resolveAnimationPlayer();
}

ct::String Ragdoll3D::getAnimationPlayerPath() const
{
    if (mPlayer) return getPathTo(mPlayer);
    return mPlayerPath;
}

void Ragdoll3D::resolveAnimationPlayer()
{
    AnimationPlayer* player = nullptr;
    if (!mPlayerPath.empty())
    {
        Node* node = getNode(mPlayerPath);
        player = node ? node->castTo<AnimationPlayer>() : nullptr;
    }
    if (player)
    {
        setAnimationPlayer(player);
        return;
    }
    const ct::String pending = mPlayerPath;
    setAnimationPlayer(nullptr);
    mPlayerPath = pending;
}

AnimationPlayer* Ragdoll3D::getAnimationPlayer() const
{
    return mPlayer;
}

void Ragdoll3D::setBone(RagdollBone slot, int bone)
{
    if (slot >= RagdollBone::Count) return;
    mBones[static_cast<std::size_t>(slot)] = bone;
    if (mRagdoll.valid() && !mActive) release();
}

int Ragdoll3D::getBone(RagdollBone slot) const
{
    return slot < RagdollBone::Count ? mBones[static_cast<std::size_t>(slot)] : -1;
}

bool Ragdoll3D::setBoneName(RagdollBone slot, const char* name)
{
    if (!mSkeleton || !name) return false;
    const int bone = mSkeleton->findBone(name);
    if (bone < 0) return false;
    setBone(slot, bone);
    return true;
}

int Ragdoll3D::autoMapBones()
{
    if (!mSkeleton) return 0;
    int found = 0;
    const int count = static_cast<int>(mSkeleton->getBoneCount());
    for (std::size_t slot = 0; slot < kSlots; ++slot)
    {
        int best = -1;
        for (int c = 0; c < 8 && kCandidates[slot].names[c] && best < 0; ++c)
        {
            for (int bone = 0; bone < count; ++bone)
            {
                const ct::String name = normalized(mSkeleton->getBoneName(bone));
                if (name == kCandidates[slot].names[c])
                {
                    best = bone;
                    break;
                }
            }
        }
        if (best >= 0)
        {
            mBones[slot] = best;
            ++found;
        }
    }
    const int left = mBones[static_cast<std::size_t>(RagdollBone::LeftUpLeg)];
    const int right = mBones[static_cast<std::size_t>(RagdollBone::RightUpLeg)];
    if (left >= 0 && right >= 0)
    {
        auto hangsFrom = [this](int ancestor, int bone)
        {
            for (int walk = bone; walk >= 0; walk = mSkeleton->getBoneParent(walk))
                if (walk == ancestor) return true;
            return false;
        };
        int hips = mBones[static_cast<std::size_t>(RagdollBone::Hips)];
        if (hips < 0 || !hangsFrom(hips, left) || !hangsFrom(hips, right))
        {
            int common = -1;
            for (int walk = mSkeleton->getBoneParent(left); walk >= 0 && common < 0;
                    walk = mSkeleton->getBoneParent(walk))
                if (hangsFrom(walk, right)) common = walk;
            if (common >= 0)
            {
                if (hips < 0) ++found;
                mBones[static_cast<std::size_t>(RagdollBone::Hips)] = common;
            }
        }
    }
    if (mRagdoll.valid() && !mActive) release();
    return found;
}

void Ragdoll3D::collectGlobalPose(ct::Vector<Math::Mat4>& global) const
{
    const int count = static_cast<int>(mSkeleton->getBoneCount());
    global.clear();
    global.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) global.push_back(mSkeleton->getBoneGlobalPose(i));
}

bool Ragdoll3D::build()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mSkeleton) return false;
    if (server->ragdollValid(mRagdoll)) return true;
    const int count = static_cast<int>(mSkeleton->getBoneCount());
    if (count <= 0) return false;
    ct::Vector<int> parents;
    ct::Vector<Math::Mat4> rest;
    ct::Vector<Math::Mat4> global;
    parents.reserve(static_cast<std::size_t>(count));
    rest.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        parents.push_back(mSkeleton->getBoneParent(i));
        rest.push_back(mSkeleton->getBoneGlobalRest(i));
    }
    collectGlobalPose(global);
    PhysicsRagdollBones map;
    map.hips = mBones[static_cast<std::size_t>(RagdollBone::Hips)];
    map.spine = mBones[static_cast<std::size_t>(RagdollBone::Spine)];
    map.neck = mBones[static_cast<std::size_t>(RagdollBone::Neck)];
    map.head = mBones[static_cast<std::size_t>(RagdollBone::Head)];
    map.leftArm = mBones[static_cast<std::size_t>(RagdollBone::LeftArm)];
    map.leftForeArm = mBones[static_cast<std::size_t>(RagdollBone::LeftForeArm)];
    map.leftHand = mBones[static_cast<std::size_t>(RagdollBone::LeftHand)];
    map.rightArm = mBones[static_cast<std::size_t>(RagdollBone::RightArm)];
    map.rightForeArm = mBones[static_cast<std::size_t>(RagdollBone::RightForeArm)];
    map.rightHand = mBones[static_cast<std::size_t>(RagdollBone::RightHand)];
    map.leftUpLeg = mBones[static_cast<std::size_t>(RagdollBone::LeftUpLeg)];
    map.leftLeg = mBones[static_cast<std::size_t>(RagdollBone::LeftLeg)];
    map.leftFoot = mBones[static_cast<std::size_t>(RagdollBone::LeftFoot)];
    map.leftToes = mBones[static_cast<std::size_t>(RagdollBone::LeftToes)];
    map.rightUpLeg = mBones[static_cast<std::size_t>(RagdollBone::RightUpLeg)];
    map.rightLeg = mBones[static_cast<std::size_t>(RagdollBone::RightLeg)];
    map.rightFoot = mBones[static_cast<std::size_t>(RagdollBone::RightFoot)];
    map.rightToes = mBones[static_cast<std::size_t>(RagdollBone::RightToes)];
    const Math::Mat4 owner = mSkeleton->getGlobalTransform();
    const float restScale = uniformScale(owner);
    PhysicsRagdollSettings settings;
    settings.scale = mScale > 0.0f ? mScale : restScale;
    settings.fatness = mFatness;
    settings.headSize = mHeadSize;
    mRagdoll = server->ragdollCreate(parents.data(), parents.size(), map, rest.data(), restScale,
            global.data(), owner, settings);
    if (!server->ragdollValid(mRagdoll))
    {
        mRagdoll = PhysicsRagdollId();
        return false;
    }
    server->ragdollSetCollisionMask(mRagdoll, mMask);
    return true;
}

bool Ragdoll3D::isBuilt() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server && server->ragdollValid(mRagdoll);
}

bool Ragdoll3D::start()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mSkeleton || mActive || !build()) return false;
    const int count = static_cast<int>(mSkeleton->getBoneCount());
    mSavedPose.clear();
    mSavedPose.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) mSavedPose.push_back(mSkeleton->getBonePose(i));
    if (!server->ragdollActivate(mRagdoll)) return false;
    mActive = true;
    if (mPlayer && mBlend >= 1.0f)
    {
        mPlayerWasActive = mPlayer->isActive();
        mPlayer->setActive(false);
    }
    return true;
}

void Ragdoll3D::stop()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mRagdoll.valid())
    {
        server->ragdollDeactivate(mRagdoll);
        if (mActive && mSkeleton)
            for (std::size_t part = 0; part < kParts; ++part)
            {
                const int bone =
                        server->ragdollGetPartBone(mRagdoll, static_cast<PhysicsRagdollPart>(part));
                if (bone >= 0 && static_cast<std::size_t>(bone) < mSavedPose.size())
                    mSkeleton->setBonePose(bone, mSavedPose[static_cast<std::size_t>(bone)]);
            }
    }
    if (mActive && mPlayer && mPlayerWasActive) mPlayer->setActive(true);
    mPlayerWasActive = false;
    mActive = false;
}

bool Ragdoll3D::isActive() const
{
    return mActive;
}

void Ragdoll3D::setBlend(float blend)
{
    mBlend = blend < 0.0f ? 0.0f : (blend > 1.0f ? 1.0f : blend);
    if (!mActive || !mPlayer) return;
    if (mBlend >= 1.0f && mPlayer->isActive())
    {
        mPlayerWasActive = true;
        mPlayer->setActive(false);
    }
    else if (mBlend < 1.0f && mPlayerWasActive && !mPlayer->isActive())
    {
        mPlayer->setActive(true);
    }
}

float Ragdoll3D::getBlend() const
{
    return mBlend;
}

void Ragdoll3D::setCollisionMask(std::uint32_t mask)
{
    mMask = mask;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mRagdoll.valid()) server->ragdollSetCollisionMask(mRagdoll, mask);
}

std::uint32_t Ragdoll3D::getCollisionMask() const
{
    return mMask;
}

void Ragdoll3D::setScale(float scale)
{
    mScale = scale < 0.0f ? 0.0f : scale;
    if (mRagdoll.valid() && !mActive) release();
}

float Ragdoll3D::getScale() const
{
    return mScale;
}

void Ragdoll3D::setFatness(float fatness)
{
    mFatness = fatness < 0.01f ? 0.01f : fatness;
    if (mRagdoll.valid() && !mActive) release();
}

float Ragdoll3D::getFatness() const
{
    return mFatness;
}

void Ragdoll3D::setHeadSize(float headSize)
{
    mHeadSize = headSize < 0.01f ? 0.01f : headSize;
    if (mRagdoll.valid() && !mActive) release();
}

float Ragdoll3D::getHeadSize() const
{
    return mHeadSize;
}

float Ragdoll3D::getFacing() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->ragdollGetFacing(mRagdoll) : 0.0f;
}

int Ragdoll3D::getPartBone(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->ragdollGetPartBone(mRagdoll, part) : -1;
}

float Ragdoll3D::getPartMass(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->ragdollGetPartMass(mRagdoll, part) : 0.0f;
}

void Ragdoll3D::applyImpulse(PhysicsRagdollPart part, const Math::Vec3& impulse,
        const Math::Vec3& offset)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mRagdoll.valid()) server->ragdollApplyImpulse(mRagdoll, part, impulse, offset);
}

void Ragdoll3D::applyBlast(const Math::Vec3& origin, float speed)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mRagdoll.valid()) return;
    for (std::size_t i = 0; i < kParts; ++i)
    {
        const PhysicsRagdollPart part = static_cast<PhysicsRagdollPart>(i);
        const Math::Vec3 center = server->ragdollGetPartTransform(mRagdoll, part).GetTranslation();
        Math::Vec3 direction = center - origin;
        direction = direction.LengthSquared() > 1e-6f ? direction.Normalized()
                                                      : Math::Vec3(0.0f, 1.0f, 0.0f);
        const float mass = server->ragdollGetPartMass(mRagdoll, part);
        server->ragdollApplyImpulse(mRagdoll, part, direction * (speed * mass),
                Math::Vec3(0.0f, 0.0f, 0.0f));
    }
}

Math::Mat4 Ragdoll3D::getPartTransform(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->ragdollGetPartTransform(mRagdoll, part) : Math::Mat4::Identity();
}

Math::Vec3 Ragdoll3D::getPartVelocity(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->ragdollGetPartVelocity(mRagdoll, part) : Math::Vec3(0.0f, 0.0f, 0.0f);
}

PhysicsRagdollId Ragdoll3D::getRagdoll() const
{
    return mRagdoll;
}

void Ragdoll3D::follow()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mSkeleton || mActive || !server->ragdollValid(mRagdoll)) return;
    ct::Vector<Math::Mat4> global;
    collectGlobalPose(global);
    server->ragdollFollow(mRagdoll, global.data(), global.size(), mSkeleton->getGlobalTransform());
}

void Ragdoll3D::writePose()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mSkeleton || !mActive || !server->ragdollIsActive(mRagdoll)) return;
    const Engine* engine = Engine::singleton();
    const float alpha =
            engine ? static_cast<float>(engine->getPhysicsInterpolationFraction()) : 1.0f;
    const int count = static_cast<int>(mSkeleton->getBoneCount());
    ct::Vector<PhysicsBonePose> poses;
    poses.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const LocalPose& pose = mSkeleton->getBonePose(i);
        PhysicsBonePose out;
        out.position = pose.position;
        out.rotation = pose.rotation;
        out.scale = pose.scale;
        poses.push_back(out);
    }
    ct::Vector<PhysicsBonePose> simulated = poses;
    server->ragdollWritePose(mRagdoll, simulated.data(), simulated.size(),
            mSkeleton->getGlobalTransform(), alpha);
    for (int i = 0; i < count; ++i)
    {
        const std::size_t k = static_cast<std::size_t>(i);
        LocalPose pose;
        if (mBlend >= 1.0f)
        {
            pose.position = simulated[k].position;
            pose.rotation = simulated[k].rotation;
            pose.scale = simulated[k].scale;
        }
        else
        {
            const float t = mBlend;
            pose.position = poses[k].position + (simulated[k].position - poses[k].position) * t;
            pose.rotation = Math::Quaternion::Slerp(poses[k].rotation, simulated[k].rotation, t)
                                    .Normalized();
            pose.scale = poses[k].scale + (simulated[k].scale - poses[k].scale) * t;
        }
        mSkeleton->setBonePose(i, pose);
    }
}

void Ragdoll3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            if (!mSkeletonPath.empty()) resolveSkeleton();
            if (!mPlayerPath.empty()) resolveAnimationPlayer();
            break;
        case kNotificationPhysicsProcess:
            follow();
            break;
        case kNotificationProcess:
            writePose();
            break;
        case kNotificationExitTree:
            stop();
            break;
        default:
            break;
    }
}

void Ragdoll3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Ragdoll3D;
    DB::addProperty<P>(PropertyInfo::nodePath("skeleton", "Skeleton3D"), &P::setSkeletonPath,
            &P::getSkeletonPath);
    DB::addProperty<P>(PropertyInfo::nodePath("animation_player", "AnimationPlayer"),
            &P::setAnimationPlayerPath, &P::getAnimationPlayerPath);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "blend", 0.0, 1.0, 0.01),
            &P::setBlend, &P::getBlend);
    DB::addProperty<P>(PropertyInfo::layers3DPhysics("collision_mask"), &P::setCollisionMask,
            &P::getCollisionMask);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "ragdoll_scale", 0.0, 100.0, 0.001, true),
            static_cast<void (P::*)(float)>(&P::setScale), &P::getScale);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "fatness", 0.01, 10.0, 0.01, true),
            &P::setFatness, &P::getFatness);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "head_size", 0.01, 10.0, 0.01, true),
            &P::setHeadSize, &P::getHeadSize);

    DB::addPropertyGroup<P>("Bones", "bones/");
    const char* names[static_cast<std::size_t>(RagdollBone::Count)] = { "bones/hips", "bones/spine",
        "bones/neck", "bones/head", "bones/left_arm", "bones/left_fore_arm", "bones/left_hand",
        "bones/right_arm", "bones/right_fore_arm", "bones/right_hand", "bones/left_up_leg",
        "bones/left_leg", "bones/left_foot", "bones/left_toes", "bones/right_up_leg",
        "bones/right_leg", "bones/right_foot", "bones/right_toes" };
    for (std::size_t i = 0; i < static_cast<std::size_t>(RagdollBone::Count); ++i)
    {
        const RagdollBone slot = static_cast<RagdollBone>(i);
        DB::addProperty<P>(
                PropertyInfo::range(PropertyType::Int, names[i], -1.0, 65535.0, 1.0, true),
                [slot](P* ragdoll, const PropertyValue& value)
                { ragdoll->setBone(slot, static_cast<int>(value.asInt())); },
                [slot](const P* ragdoll)
                { return PropertyValue::fromInt(ragdoll->getBone(slot)); });
    }
}

} // namespace prisma3d
