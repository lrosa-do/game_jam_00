#include "character/Ragdoll.h"

#include "dynamics/PhysicsWorld.h"

#include <ct/array.hpp>
#include <ct/vector.hpp>
#include <kinetix/core/Math.h>
#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr usize kPartCount = static_cast<usize>(RagdollPart::Count);

constexpr RagdollPart kParent[kPartCount] = {
    RagdollPart::Count,
    RagdollPart::Pelvis,
    RagdollPart::Spine,
    RagdollPart::Pelvis,
    RagdollPart::LeftUpperLeg,
    RagdollPart::Pelvis,
    RagdollPart::RightUpperLeg,
    RagdollPart::Spine,
    RagdollPart::LeftUpperArm,
    RagdollPart::Spine,
    RagdollPart::RightUpperArm,
    RagdollPart::LeftLowerLeg,
    RagdollPart::RightLowerLeg,
};

constexpr f32 kTwistAngle[kPartCount] = { 0.0f, 5.0f, 90.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f,
    45.0f, 45.0f, 45.0f, 20.0f, 20.0f };

constexpr f32 kNormalAngle[kPartCount] = { 0.0f, 40.0f, 45.0f, 45.0f, 0.0f, 45.0f, 0.0f, 90.0f,
    0.0f, 90.0f, 0.0f, 20.0f, 20.0f };

constexpr f32 kPlaneAngle[kPartCount] = { 0.0f, 40.0f, 45.0f, 45.0f, 60.0f, 45.0f, 60.0f, 45.0f,
    90.0f, 45.0f, 90.0f, 20.0f, 20.0f };

constexpr f32 kMinMassRatio = 0.8f;
constexpr f32 kMaxMassRatio = 1.2f;
constexpr f32 kMaxInertiaIncrease = 2.0f;

Math::mat4 composeLocal(const LocalPose& pose)
{
    return Math::translate(Math::mat4(1.0f), pose.position) * Math::mat4_cast(pose.rotation) *
           Math::scale(Math::mat4(1.0f), pose.scale);
}

Math::quat rotationBetween(const Math::vec3& from, const Math::vec3& to)
{
    const f32 cosine = Math::clamp(Math::dot(from, to), -1.0f, 1.0f);
    if (cosine > 0.9999f) return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (cosine < -0.9999f)
    {
        const Math::vec3 orthogonal =
                fabsf(from.x) < 0.9f ? Math::vec3(1.0f, 0.0f, 0.0f) : Math::vec3(0.0f, 0.0f, 1.0f);
        return Math::angleAxis(Math::pi<f32>(), Math::normalize(Math::cross(from, orthogonal)));
    }
    const Math::vec3 axis = Math::cross(from, to);
    const f32 scale = sqrtf((1.0f + cosine) * 2.0f);
    return Math::normalize(Math::quat(scale * 0.5f, axis / scale));
}

bool decomposeRigid(const Math::mat4& transform, Math::vec3& position, Math::quat& rotation)
{
    Math::vec3 scale;
    Math::vec3 skew;
    Math::vec4 perspective;
    if (!Math::decompose(transform, scale, rotation, position, skew, perspective)) return false;
    rotation = Math::normalize(rotation);
    return true;
}

bool isLowerLeg(RagdollPart part)
{
    return part == RagdollPart::LeftLowerLeg || part == RagdollPart::RightLowerLeg;
}

bool isLowerArm(RagdollPart part)
{
    return part == RagdollPart::LeftLowerArm || part == RagdollPart::RightLowerArm;
}

bool isFoot(RagdollPart part)
{
    return part == RagdollPart::LeftFoot || part == RagdollPart::RightFoot;
}

} // namespace

Ragdoll::~Ragdoll()
{
    removeFromWorld();
}

bool Ragdoll::build(const SkeletonView& skeleton, const RagdollBoneMap& bones,
        const ct::Vector<Math::mat4>& restPose, f32 restScale, const RagdollSettings& settings)
{
    mValid = false;
    if (mWorld) return false;
    const u32 count = skeleton.boneCount();
    if (count == 0 || restPose.size() < count || restScale <= 0.0f) return false;

    auto valid = [count](s32 bone) -> s32
    { return bone >= 0 && static_cast<u32>(bone) < count ? bone : -1; };

    const s32 neck = valid(bones.neck);
    const s32 head = valid(bones.head);
    const s32 headBone = neck >= 0 ? neck : head;
    struct BonePair
    {
        s32 near;
        s32 far;
    };
    const BonePair pairs[kPartCount] = {
        { valid(bones.hips), valid(bones.spine) },
        { valid(bones.spine), headBone },
        { headBone, neck >= 0 ? head : -1 },
        { valid(bones.leftUpLeg), valid(bones.leftLeg) },
        { valid(bones.leftLeg), valid(bones.leftFoot) },
        { valid(bones.rightUpLeg), valid(bones.rightLeg) },
        { valid(bones.rightLeg), valid(bones.rightFoot) },
        { valid(bones.leftArm), valid(bones.leftForeArm) },
        { valid(bones.leftForeArm), valid(bones.leftHand) },
        { valid(bones.rightArm), valid(bones.rightForeArm) },
        { valid(bones.rightForeArm), valid(bones.rightHand) },
        { valid(bones.leftFoot), valid(bones.leftToes) },
        { valid(bones.rightFoot), valid(bones.rightToes) },
    };
    for (usize c = 0; c < kPartCount; ++c)
    {
        if (pairs[c].near < 0) return false;
        const RagdollPart part = static_cast<RagdollPart>(c);
        if (pairs[c].far < 0 && part != RagdollPart::Head && !isFoot(part)) return false;
    }

    mSettings = settings;
    const f32 scale = settings.scale;

    auto scaledRest = [&](s32 bone) -> Math::mat4
    {
        Math::mat4 rest = restPose[static_cast<usize>(bone)];
        rest[3] = Math::vec4(Math::vec3(rest[3]) * restScale, 1.0f);
        return rest;
    };

    const Math::vec3 leftShoulder(scaledRest(bones.leftArm)[3]);
    const Math::vec3 rightShoulder(scaledRest(bones.rightArm)[3]);
    mFacing = rightShoulder.x < leftShoulder.x ? 1.0f : -1.0f;
    const Math::vec3 forward(0.0f, 0.0f, mFacing);

    for (usize c = 0; c < kPartCount; ++c)
    {
        const RagdollPart partId = static_cast<RagdollPart>(c);
        Part& p = mParts[c];
        p.boneIndex = pairs[c].near;
        p.parentPart = kParent[c];
        p.bodyId = 0xFFFFFFFFu;
        p.hasTarget = false;

        const Math::mat4 restA = scaledRest(pairs[c].near);
        const Math::vec3 rootA(restA[3]);
        Math::vec3 rootB = rootA;
        if (pairs[c].far >= 0) rootB = Math::vec3(scaledRest(pairs[c].far)[3]);
        else if (isFoot(partId))
        {
            const BonePair& shin = pairs[partId == RagdollPart::LeftFoot
                                                 ? static_cast<usize>(RagdollPart::LeftLowerLeg)
                                                 : static_cast<usize>(RagdollPart::RightLowerLeg)];
            const f32 shinLength = Math::length(Math::vec3(scaledRest(shin.far)[3]) -
                                                Math::vec3(scaledRest(shin.near)[3]));
            rootB = rootA + forward * (shinLength * 0.55f);
        }
        const Math::vec3 span = rootB - rootA;
        const f32 spanLength = Math::length(span);

        f32 mass = scale * settings.massScale;
        f32 capsuleHeight = scale;
        f32 capsuleRadius = scale * settings.fatness;
        if (partId == RagdollPart::Head)
        {
            capsuleHeight = 0.05f * scale;
            capsuleRadius = 0.1f * scale * settings.headSize;
        }
        else
        {
            capsuleHeight = spanLength;
            switch (partId)
            {
                case RagdollPart::Pelvis:
                    capsuleRadius = 0.1f * scale * settings.fatness;
                    break;
                case RagdollPart::Spine:
                    capsuleRadius = 0.1f * scale * settings.fatness;
                    capsuleHeight -= capsuleRadius * 2.0f;
                    break;
                case RagdollPart::LeftLowerArm:
                case RagdollPart::RightLowerArm:
                    capsuleRadius = capsuleHeight * 0.15f * settings.fatness;
                    capsuleHeight += capsuleRadius;
                    break;
                case RagdollPart::LeftUpperLeg:
                case RagdollPart::RightUpperLeg:
                    capsuleRadius = capsuleHeight * 0.15f * settings.fatness;
                    capsuleHeight -= capsuleRadius * 2.0f;
                    break;
                case RagdollPart::LeftLowerLeg:
                case RagdollPart::RightLowerLeg:
                    capsuleRadius = capsuleHeight * 0.15f * settings.fatness;
                    capsuleHeight -= capsuleRadius;
                    break;
                case RagdollPart::LeftFoot:
                case RagdollPart::RightFoot:
                    capsuleRadius = capsuleHeight * 0.3f * settings.fatness;
                    break;
                default:
                    capsuleRadius = capsuleHeight * 0.2f * settings.fatness;
                    capsuleHeight -= capsuleRadius * 2.0f;
                    break;
            }
        }
        capsuleRadius = fabsf(capsuleRadius);
        capsuleHeight = fabsf(capsuleHeight);

        Math::vec3 axis = forward;
        if (isFoot(partId))
        {
            const Math::vec3 flat(span.x, 0.0f, span.z);
            if (Math::length(flat) > 1.0e-5f) axis = Math::normalize(flat);
        }
        else if (spanLength > 1.0e-5f)
            axis = Math::normalize(span);
        else
            axis = Math::vec3(0.0f, 1.0f, 0.0f);
        p.axis = axis;

        const f32 offset = capsuleHeight * 0.5f + capsuleRadius;
        if (partId == RagdollPart::Pelvis) p.offset = Math::vec3(0.0f);
        else if (isFoot(partId))
            p.offset = Math::vec3(0.0f, span.y, 0.0f) + axis * (capsuleHeight * 0.5f);
        else
            p.offset = axis * offset;

        if (isFoot(partId))
        {
            p.box = BoxShape(Math::vec3(capsuleRadius, capsuleRadius * 0.75f,
                    capsuleHeight * 0.5f + capsuleRadius));
            p.shape = &p.box;
            p.shapeRotation = rotationBetween(Math::vec3(0.0f, 0.0f, 1.0f), axis);
        }
        else
        {
            p.capsule = CapsuleShape(capsuleRadius, capsuleHeight * 0.5f);
            p.shape = &p.capsule;
            p.shapeRotation = rotationBetween(Math::vec3(0.0f, 1.0f, 0.0f), axis);
        }

        Math::vec3 restTranslation;
        if (!decomposeRigid(restA, restTranslation, p.restBasis)) return false;
        p.restPosition = rootA;
        p.restBodyPosition = rootA + p.offset;

        p.mass = mass;
        p.inertia = p.shape->inertia(mass);
    }

    stabilize();
    resolveChains(skeleton);

    for (Part& p: mParts)
    {
        RigidBody& body = p.rigidBody;
        body.setBodyType(BodyType::Dynamic);
        body.setMass(p.mass);
        body.setInertiaTensor(p.inertia);
        body.setDamping(0.95f, 0.95f);
        body.setPosition(p.restBodyPosition);
        body.setOrientation(p.shapeRotation);
        body.setVelocity(Math::vec3(0.0f));
        body.setAngularVelocity(Math::vec3(0.0f));
        body.calculateDerivedData();
        body.setBodyType(BodyType::Kinematic);
    }

    mValid = true;
    return true;
}

void Ragdoll::stabilize()
{
    ct::Array<f32, kPartCount> massRatio;
    massRatio[0] = 1.0f;
    f32 totalMassRatio = 1.0f;
    for (usize i = 1; i < kPartCount; ++i)
    {
        const usize parent = static_cast<usize>(kParent[i]);
        const f32 ratio = mParts[i].mass / mParts[parent].mass;
        massRatio[i] = massRatio[parent] * Math::clamp(ratio, kMinMassRatio, kMaxMassRatio);
        totalMassRatio += massRatio[i];
    }

    f32 totalMass = 0.0f;
    for (const Part& p: mParts) totalMass += p.mass;
    const f32 ratioToMass = totalMass / totalMassRatio;

    for (usize i = 0; i < kPartCount; ++i)
    {
        Part& p = mParts[i];
        const f32 oldMass = p.mass;
        const f32 newMass = massRatio[i] * ratioToMass;
        p.mass = newMass;
        p.inertia = p.inertia * (newMass / oldMass);
    }

    ct::Array<Math::vec3, kPartCount> diagonal;
    ct::Array<f32, kPartCount> childSum;
    for (usize i = 0; i < kPartCount; ++i)
    {
        diagonal[i] = Math::vec3(mParts[i].inertia[0][0], mParts[i].inertia[1][1],
                mParts[i].inertia[2][2]);
        childSum[i] = 0.0f;
    }
    for (usize i = kPartCount - 1; i > 0; --i)
    {
        const usize parent = static_cast<usize>(kParent[i]);
        childSum[parent] += diagonal[i][0] + childSum[i];
    }
    for (usize i = 0; i < kPartCount; ++i)
    {
        if (childSum[i] == 0.0f) continue;
        const f32 minimum = Math::min(kMaxInertiaIncrease * diagonal[i][0], childSum[i]);
        const Math::vec3 adjusted(Math::max(diagonal[i].x, minimum),
                Math::max(diagonal[i].y, minimum), Math::max(diagonal[i].z, minimum));
        Math::mat3 inertia(0.0f);
        inertia[0][0] = adjusted.x;
        inertia[1][1] = adjusted.y;
        inertia[2][2] = adjusted.z;
        mParts[i].inertia = inertia;
    }
}

void Ragdoll::resolveChains(const SkeletonView& skeleton)
{
    for (usize i = 0; i < kPartCount; ++i)
    {
        Part& p = mParts[i];
        p.parentChain.clear();
        p.rooted = true;
        const s32 parentBone =
                p.parentPart == RagdollPart::Count ? -1 : part(p.parentPart).boneIndex;
        s32 walk = skeleton.boneParent(static_cast<u32>(p.boneIndex));
        while (walk >= 0)
        {
            if (walk == parentBone)
            {
                p.rooted = false;
                break;
            }
            p.parentChain.push_back(walk);
            walk = skeleton.boneParent(static_cast<u32>(walk));
        }
    }
}

void Ragdoll::bodyFromBone(const Part& p, const Math::vec3& bonePosition,
        const Math::quat& boneRotation, Math::vec3& position, Math::quat& rotation) const
{
    const Math::quat toRest = Math::normalize(boneRotation * Math::conjugate(p.restBasis));
    rotation = Math::normalize(toRest * p.shapeRotation);
    position = bonePosition + toRest * p.offset;
}

void Ragdoll::boneTransform(const Part& p, Math::vec3& position, Math::quat& rotation) const
{
    const RigidBody& body = p.rigidBody;
    const Math::quat toRest =
            Math::normalize(body.orientation() * Math::conjugate(p.shapeRotation));
    rotation = Math::normalize(toRest * p.restBasis);
    position = body.position() - toRest * p.offset;
}

void Ragdoll::createJoints()
{
    const Math::vec3 forward(0.0f, 0.0f, mFacing);
    for (usize i = 1; i < kPartCount; ++i)
    {
        Part& p = mParts[i];
        Part& parentP = part(p.parentPart);
        const RagdollPart partId = static_cast<RagdollPart>(i);

        if (isLowerLeg(partId) || isLowerArm(partId))
        {
            Math::vec3 axis = isLowerLeg(partId) ? Math::cross(forward, parentP.axis)
                                                 : Math::cross(parentP.axis, forward);
            if (Math::length(axis) <= 1.0e-5f) axis = Math::vec3(mFacing, 0.0f, 0.0f);
            axis = Math::normalize(axis);
            mHingeJoints.emplace_back(parentP.rigidBody, p.rigidBody, p.restPosition, axis);
            mHingeJoints.back().setLimits(0.0f,
                    Math::pi<f32>() * (isLowerLeg(partId) ? 0.8f : 0.6f));
            p.joint = &mHingeJoints.back();
            continue;
        }

        const Math::vec3 twist = Math::normalize(p.restBodyPosition - p.restPosition);
        const Math::vec3 plane =
                isFoot(partId) ? Math::vec3(mFacing, 0.0f, 0.0f) : Math::vec3(0.0f, 0.0f, mFacing);
        mConeTwistJoints.emplace_back(parentP.rigidBody, p.rigidBody, p.restPosition, twist, plane);
        mConeTwistJoints.back().setSwingLimits(Math::radians(kNormalAngle[i]),
                Math::radians(kPlaneAngle[i]));
        mConeTwistJoints.back().setTwistLimits(-Math::radians(kTwistAngle[i]),
                Math::radians(kTwistAngle[i]));
        p.joint = &mConeTwistJoints.back();
    }
}

void Ragdoll::addToWorld(PhysicsWorld& world, const Math::mat4& ownerWorld,
        const ct::Vector<Math::mat4>& globalPose)
{
    if (!mValid || mWorld) return;

    mConeTwistJoints.clear();
    mHingeJoints.clear();
    for (Part& p: mParts)
    {
        p.joint = nullptr;
        p.detached = false;
        RigidBody& body = p.rigidBody;
        body.setPosition(p.restBodyPosition);
        body.setOrientation(p.shapeRotation);
        body.setVelocity(Math::vec3(0.0f));
        body.setAngularVelocity(Math::vec3(0.0f));
        body.calculateDerivedData();
    }
    createJoints();

    setTargetPose(ownerWorld, globalPose);
    for (Part& p: mParts)
    {
        RigidBody& body = p.rigidBody;
        body.setBodyType(BodyType::Kinematic);
        if (p.hasTarget)
        {
            body.setPosition(p.targetPosition);
            body.setOrientation(p.targetRotation);
        }
        p.hasTarget = false;
        body.setVelocity(Math::vec3(0.0f));
        body.setAngularVelocity(Math::vec3(0.0f));
        body.calculateDerivedData();
        body.setAwake(true);
        boneTransform(p, p.previousPosition, p.previousRotation);

        BodyEntry entry;
        entry.body = &body;
        entry.shape = p.shape;
        entry.friction = 0.2f;
        entry.restitution = 0.0f;
        entry.filter = mFilter;
        p.bodyId = world.addBody(entry);
    }

    for (usize i = 1; i < kPartCount; ++i)
        world.setPairIgnored(mParts[i].bodyId, part(mParts[i].parentPart).bodyId, true);

    for (ConeTwistJoint& joint: mConeTwistJoints)
    {
        joint.resetWarmStart();
        world.addJoint(&joint);
    }
    for (HingeJoint& joint: mHingeJoints)
    {
        joint.resetWarmStart();
        world.addJoint(&joint);
    }

    mWorld = &world;
    mActive = false;
}

void Ragdoll::removeFromWorld()
{
    if (mWorld)
    {
        for (ConeTwistJoint& joint: mConeTwistJoints) mWorld->removeJoint(&joint);
        for (HingeJoint& joint: mHingeJoints) mWorld->removeJoint(&joint);
        for (Part& p: mParts)
            if (p.bodyId != 0xFFFFFFFFu)
            {
                mWorld->removeBody(p.bodyId);
                p.bodyId = 0xFFFFFFFFu;
            }
    }
    mConeTwistJoints.clear();
    mHingeJoints.clear();
    for (Part& p: mParts)
    {
        p.joint = nullptr;
        p.detached = false;
    }
    mWorld = nullptr;
    mActive = false;
}

void Ragdoll::activate()
{
    if (!mWorld || mActive) return;
    for (Part& p: mParts)
    {
        p.rigidBody.setBodyType(BodyType::Dynamic);
        p.rigidBody.setAwake(true);
        p.hasTarget = false;
    }
    mActive = true;
}

bool Ragdoll::detach(RagdollPart part)
{
    if (!mWorld || part >= RagdollPart::Count) return false;
    Part& p = mParts[static_cast<usize>(part)];
    if (p.detached || !p.joint) return false;
    mWorld->removeJoint(p.joint);
    mWorld->setPairIgnored(p.bodyId, this->part(p.parentPart).bodyId, false);
    p.detached = true;
    return true;
}

void Ragdoll::attachAll()
{
    for (usize i = 1; i < kPartCount; ++i)
    {
        Part& p = mParts[i];
        if (!p.detached) continue;
        const RagdollPart partId = static_cast<RagdollPart>(i);
        if (isLowerLeg(partId) || isLowerArm(partId))
            static_cast<HingeJoint*>(p.joint)->resetWarmStart();
        else
            static_cast<ConeTwistJoint*>(p.joint)->resetWarmStart();
        mWorld->addJoint(p.joint);
        mWorld->setPairIgnored(p.bodyId, part(p.parentPart).bodyId, true);
        p.detached = false;
    }
}

void Ragdoll::deactivate()
{
    if (!mWorld || !mActive) return;
    attachAll();
    for (Part& p: mParts)
    {
        p.rigidBody.setBodyType(BodyType::Kinematic);
        p.rigidBody.setVelocity(Math::vec3(0.0f));
        p.rigidBody.setAngularVelocity(Math::vec3(0.0f));
        p.hasTarget = false;
    }
    mActive = false;
}

void Ragdoll::setTargetPose(const Math::mat4& ownerWorld, const ct::Vector<Math::mat4>& globalPose)
{
    if (!mValid || mActive) return;
    for (Part& p: mParts)
    {
        p.hasTarget = false;
        if (p.boneIndex < 0 || static_cast<usize>(p.boneIndex) >= globalPose.size()) continue;
        const Math::mat4 boneWorld = ownerWorld * globalPose[static_cast<usize>(p.boneIndex)];
        Math::vec3 bonePosition;
        Math::quat boneRotation;
        if (!decomposeRigid(boneWorld, bonePosition, boneRotation)) continue;
        bodyFromBone(p, bonePosition, boneRotation, p.targetPosition, p.targetRotation);
        p.hasTarget = true;
    }
}

void Ragdoll::beginStep(f32 duration)
{
    if (!mWorld) return;
    for (Part& p: mParts)
    {
        boneTransform(p, p.previousPosition, p.previousRotation);
        if (mActive) continue;

        RigidBody& body = p.rigidBody;
        if (!p.hasTarget || duration <= 0.0f)
        {
            body.setVelocity(Math::vec3(0.0f));
            body.setAngularVelocity(Math::vec3(0.0f));
            continue;
        }
        body.setVelocity((p.targetPosition - body.position()) / duration);
        Math::quat change = Math::normalize(p.targetRotation * Math::conjugate(body.orientation()));
        if (change.w < 0.0f) change = -change;
        const Math::vec3 vector(change.x, change.y, change.z);
        const f32 vectorLength = Math::length(vector);
        Math::vec3 angular(0.0f);
        if (vectorLength > 1.0e-7f)
        {
            const f32 angle = 2.0f * atan2f(vectorLength, change.w);
            angular = vector * (angle / vectorLength / duration);
        }
        body.setAngularVelocity(angular);
        body.setAwake(true);
    }
}

void Ragdoll::endStep()
{
    if (!mWorld || mActive) return;
    for (Part& p: mParts)
    {
        if (!p.hasTarget) continue;
        p.rigidBody.setPosition(p.targetPosition);
        p.rigidBody.setOrientation(p.targetRotation);
        p.rigidBody.calculateDerivedData();
        p.hasTarget = false;
    }
}

void Ragdoll::writePose(ct::Vector<LocalPose>& localPose, const Math::mat4& ownerWorld,
        f32 alpha) const
{
    if (!mWorld || !mActive) return;

    const f32 t = Math::clamp(alpha, 0.0f, 1.0f);
    const Math::mat4 invOwner = Math::inverse(ownerWorld);
    Math::vec3 ownerPosition;
    Math::quat ownerRotation;
    if (!decomposeRigid(ownerWorld, ownerPosition, ownerRotation)) return;
    const Math::quat invOwnerRotation = Math::conjugate(ownerRotation);

    ct::Array<Math::mat4, kPartCount> modelOf;
    modelOf.fill(Math::mat4(1.0f));

    for (usize i = 0; i < kPartCount; ++i)
    {
        const Part& p = mParts[i];
        if (p.boneIndex < 0 || static_cast<usize>(p.boneIndex) >= localPose.size()) continue;

        Math::vec3 position;
        Math::quat rotation;
        boneTransform(p, position, rotation);
        position = Math::mix(p.previousPosition, position, t);
        rotation = Math::normalize(Math::slerp(p.previousRotation, rotation, t));

        const Math::vec3 modelPosition(invOwner * Math::vec4(position, 1.0f));
        const Math::quat modelRotation = Math::normalize(invOwnerRotation * rotation);
        const Math::mat4 boneModel =
                Math::translate(Math::mat4(1.0f), modelPosition) * Math::mat4_cast(modelRotation);
        modelOf[i] = boneModel;

        Math::mat4 parentModel =
                p.rooted ? Math::mat4(1.0f) : modelOf[static_cast<usize>(p.parentPart)];
        for (auto it = p.parentChain.rbegin(); it != p.parentChain.rend(); ++it)
            parentModel = parentModel * composeLocal(localPose[static_cast<usize>(*it)]);

        const Math::mat4 local = Math::inverse(parentModel) * boneModel;
        LocalPose pose;
        Math::vec3 skew;
        Math::vec4 perspective;
        if (!Math::decompose(local, pose.scale, pose.rotation, pose.position, skew, perspective))
            continue;
        pose.rotation = Math::normalize(pose.rotation);
        localPose[static_cast<usize>(p.boneIndex)] = pose;
    }
}

void Ragdoll::setCollisionMask(u32 mask)
{
    mFilter.mask = mask;
    if (!mWorld) return;
    for (const Part& p: mParts)
        if (BodyEntry* entry = mWorld->body(p.bodyId)) entry->filter = mFilter;
}

} // namespace kinetix::Physics
