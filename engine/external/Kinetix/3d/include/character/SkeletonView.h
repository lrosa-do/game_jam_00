#ifndef KINETIX_PHYSICS_CHARACTER_SKELETON_VIEW_H
#define KINETIX_PHYSICS_CHARACTER_SKELETON_VIEW_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Per-bone local transform relative to the parent bone. Ragdoll only reads
// and writes these fields, so a bridge converts to/from its own pose type
// with a plain copy.
struct LocalPose
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

// What Ragdoll::build() needs from a skeleton to walk the parent chain
// between two already-resolved bones. Resolving which bone is "Hips" etc. is
// the bridge's job (RagdollBoneMap); only indices are compared here.
class SkeletonView
{
public:
    virtual ~SkeletonView() = default;

    virtual u32 boneCount() const = 0;
    // -1 for a root bone.
    virtual s32 boneParent(u32 index) const = 0;
};

// The bones of a humanoid ragdoll, resolved to indices into the skeleton a
// SkeletonView wraps; -1 means "not found". The neck is optional (the head
// body falls back to the head bone); the others are required, hands and toes
// included, since they give the far end of the forearm and foot bodies.
struct RagdollBoneMap
{
    s32 hips = -1;
    s32 spine = -1;
    s32 neck = -1;
    s32 head = -1;
    s32 leftArm = -1;
    s32 leftForeArm = -1;
    s32 leftHand = -1;
    s32 rightArm = -1;
    s32 rightForeArm = -1;
    s32 rightHand = -1;
    s32 leftUpLeg = -1;
    s32 leftLeg = -1;
    s32 leftFoot = -1;
    s32 leftToes = -1;
    s32 rightUpLeg = -1;
    s32 rightLeg = -1;
    s32 rightFoot = -1;
    s32 rightToes = -1;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_CHARACTER_SKELETON_VIEW_H
