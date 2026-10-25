#pragma once

#include "mathc.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct LocalPose
{
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion rotation = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);

    Math::Mat4 toMatrix() const;
    static bool fromMatrix(const Math::Mat4& matrix, LocalPose& out);
};

struct Bone
{
    ct::String name;
    int parent = -1;
    Math::Mat4 bindLocal = Math::Mat4::Identity();
    Math::Mat4 inverseBind = Math::Mat4::Identity();
};

class Skeleton
{
public:
    bool empty() const;
    std::uint32_t boneCount() const;
    const Bone& bone(std::uint32_t index) const;
    int findBone(const char* name) const;
    bool addBone(const ct::String& name, int parent, const Math::Mat4& bindLocal,
            const Math::Mat4& inverseBind);
    bool finalize();
    bool finalized() const;
    void scalePositions(float scale);
    const ct::Vector<std::uint16_t>& order() const;

    void bindPose(ct::Vector<LocalPose>& pose) const;
    void evaluate(const ct::Vector<LocalPose>& localPose, ct::Vector<Math::Mat4>& globalPose,
            ct::Vector<Math::Mat4>& palette) const;

private:
    ct::Vector<Bone> mBones;
    ct::Vector<std::uint16_t> mOrder;
};

struct BoneTrack
{
    int bone = -1;
    ct::Vector<float> times;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Quaternion> rotations;
    ct::Vector<Math::Vec3> scales;
};

struct NodeTrack
{
    ct::String path;
    ct::Vector<float> positionTimes;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<float> rotationTimes;
    ct::Vector<Math::Quaternion> rotations;
    ct::Vector<float> scaleTimes;
    ct::Vector<Math::Vec3> scales;

    bool samplePosition(float time, Math::Vec3& out) const;
    bool sampleRotation(float time, Math::Quaternion& out) const;
    bool sampleScale(float time, Math::Vec3& out) const;
};

struct MethodTrack
{
    ct::String path;
    ct::Vector<float> times;
    ct::Vector<ct::String> methods;
};

struct ValueTrack
{
    ct::String path;
    ct::String property;
    ct::Vector<float> times;
    ct::Vector<float> values;

    bool sample(float time, float& out) const;
};

struct AnimationTrack
{
    ct::String path;
    ct::Vector<float> times;
    ct::Vector<ct::String> animations;
};

enum class LoopedFlag : std::uint8_t
{
    None,
    End,
    Start
};

enum class LoopMode : std::uint8_t
{
    None,
    Linear,
    PingPong
};

struct AnimationEvent
{
    float time = 0.0f;
    ct::String name;
};

class AnimationClip
{
public:
    const ct::String& name() const;
    void setName(const ct::String& name);
    float duration() const;
    void setDuration(float duration);
    int skeleton() const;
    void setSkeleton(int skeleton);
    LoopMode loopMode() const;
    void setLoopMode(LoopMode mode);

    ct::Vector<BoneTrack>& tracks();
    const ct::Vector<BoneTrack>& tracks() const;

    ct::Vector<NodeTrack>& nodeTracks();
    const ct::Vector<NodeTrack>& nodeTracks() const;
    int addNodeTrack(const ct::String& path);
    int findNodeTrack(const ct::String& path) const;
    int positionTrackInsertKey(int track, float time, const Math::Vec3& position);
    int rotationTrackInsertKey(int track, float time, const Math::Quaternion& rotation);
    int scaleTrackInsertKey(int track, float time, const Math::Vec3& scale);
    ct::Vector<MethodTrack>& methodTracks();
    const ct::Vector<MethodTrack>& methodTracks() const;
    int addMethodTrack(const ct::String& path);
    int methodTrackInsertKey(int track, float time, const ct::String& method);

    ct::Vector<ValueTrack>& valueTracks();
    const ct::Vector<ValueTrack>& valueTracks() const;
    int addValueTrack(const ct::String& path, const ct::String& property);
    int valueTrackInsertKey(int track, float time, float value);

    ct::Vector<AnimationTrack>& animationTracks();
    const ct::Vector<AnimationTrack>& animationTracks() const;
    int addAnimationTrack(const ct::String& path);
    int animationTrackInsertKey(int track, float time, const ct::String& animation);

    void keyIndicesInRange(const ct::Vector<float>& times, double time, double delta,
            LoopedFlag loopedFlag, ct::Vector<int>& indices) const;
    static int findKey(const ct::Vector<float>& times, double time, bool exact);

    int insertNodeKey(const ct::String& path, float time, const LocalPose& pose,
            bool position = true, bool rotation = true, bool scale = true);
    AnimationClip extract(float first, float last) const;

    void sample(float time, ct::Vector<LocalPose>& pose) const;
    void setKeyframe(int bone, float time, const LocalPose& pose);
    void removeKeyframe(int bone, float time);

    void addEvent(float time, const ct::String& name);
    void removeEvent(const ct::String& name);
    void clearEvents();
    const ct::Vector<AnimationEvent>& events() const;

private:
    ct::String mName;
    float mDuration = 0.0f;
    int mSkeleton = 0;
    LoopMode mLoopMode = LoopMode::None;
    ct::Vector<BoneTrack> mTracks;
    ct::Vector<NodeTrack> mNodeTracks;
    ct::Vector<MethodTrack> mMethodTracks;
    ct::Vector<ValueTrack> mValueTracks;
    ct::Vector<AnimationTrack> mAnimationTracks;
    ct::Vector<AnimationEvent> mEvents;
};

} // namespace prisma3d
