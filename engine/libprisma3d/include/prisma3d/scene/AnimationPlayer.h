#pragma once

#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/Node.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Node3D;
class Skeleton3D;

class AnimationPlayer : public Node
{
public:
    PRISMA3D_NODE_CLASS(AnimationPlayer, Node);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    enum class CallbackModeProcess : std::uint8_t
    {
        Physics,
        Idle,
        Manual
    };

    AnimationPlayer();
    ~AnimationPlayer() override;

    bool addAnimation(const ct::String& name, const AnimationClip* clip);
    bool removeAnimation(const ct::String& name);
    bool hasAnimation(const ct::String& name) const;
    const AnimationClip* getAnimation(const ct::String& name) const;
    std::size_t getAnimationCount() const;
    const ct::String& getAnimationName(std::size_t index) const;
    int importAnimations(const ct::String& path, const ct::String& prefix = ct::String());
    bool addAnimationCopy(const ct::String& name, const AnimationClip& clip);
    ct::String getAnimationNames() const;
    void setAnimationSource(const ct::String& path);
    void setAnimationFiles(const ct::String& files);
    const ct::String& getAnimationFiles() const;

    void setSkeletonPath(const ct::String& path);
    const ct::String& getSkeletonPath() const;
    Skeleton3D* getSkeleton() const;
    void setRootNode(const ct::String& path);
    const ct::String& getRootNode() const;
    int recordKey(AnimationClip& clip, const Node3D* node, float time, bool position = true,
            bool rotation = true, bool scale = true) const;

    bool setAnimationLoop(const ct::String& animation, bool loop);
    bool getAnimationLoop(const ct::String& animation) const;
    void animationSetNext(const ct::String& animation, const ct::String& next);
    const ct::String& animationGetNext(const ct::String& animation) const;
    void setBlendTime(const ct::String& from, const ct::String& to, double time);
    double getBlendTime(const ct::String& from, const ct::String& to) const;
    void setDefaultBlendTime(double time);
    double getDefaultBlendTime() const;

    void play(const ct::String& name = ct::String(), double customBlend = -1.0,
            float customScale = 1.0f, bool fromEnd = false);
    void playBackwards(const ct::String& name = ct::String(), double customBlend = -1.0);
    void queue(const ct::String& name);
    std::size_t getQueueCount() const;
    const ct::String& getQueued(std::size_t index) const;
    void clearQueue();
    void pause();
    void stop(bool keepState = false);
    bool isPlaying() const;
    const ct::String& getCurrentAnimation() const;
    const ct::String& getAssignedAnimation() const;
    void setSpeedScale(float speed);
    float getSpeedScale() const;
    float getPlayingSpeed() const;
    void setAutoplay(const ct::String& name);
    const ct::String& getAutoplay() const;
    void seek(double time, bool update = false);
    double getCurrentAnimationPosition() const;
    double getCurrentAnimationLength() const;
    void setActive(bool active);
    bool isActive() const;
    void setCallbackModeProcess(CallbackModeProcess mode);
    CallbackModeProcess getCallbackModeProcess() const;
    void setDeterministic(bool deterministic);
    bool isDeterministic() const;
    void advance(double delta);

    ct::Function<void(const ct::String&)> animationStarted;
    ct::Function<void(const ct::String&)> animationFinished;
    ct::Function<void(const ct::String&, const ct::String&)> animationChanged;
    ct::Function<void(Node*, const ct::String&)> methodCalled;
    ct::Function<void(Node*, const ct::String&, float)> valueChanged;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    struct Entry
    {
        ct::String name;
        const AnimationClip* clip = nullptr;
        bool fromFile = false;
    };

    struct BlendTime
    {
        ct::String from;
        ct::String to;
        double time = 0.0;
    };

    struct NextEntry
    {
        ct::String from;
        ct::String next;
    };

    struct PlaybackData
    {
        bool enabled = false;
        ct::String name;
        const AnimationClip* clip = nullptr;
        double length = 0.0;
        double pos = 0.0;
        float speedScale = 1.0f;
    };

    struct Blend
    {
        PlaybackData data;
        double blendTime = 0.0;
        double blendLeft = 0.0;
    };

    struct Instance
    {
        const AnimationClip* clip = nullptr;
        double time = 0.0;
        double delta = 0.0;
        float weight = 0.0f;
        LoopedFlag loopedFlag = LoopedFlag::None;
        bool seeked = false;
        bool started = false;
    };

    struct ValueCache
    {
        Node* node = nullptr;
        const ValueTrack* first = nullptr;
        float init = 0.0f;
        float value = 0.0f;
        float totalWeight = 0.0f;
        std::size_t weightPass = 0;
    };

    struct ValueTrackRef
    {
        const ValueTrack* track = nullptr;
        std::size_t cache = 0;
    };

    struct PendingCall
    {
        Node* node = nullptr;
        ct::String method;
    };

    struct TrackCache
    {
        LocalPose init;
        Math::Vec3 position;
        Math::Quaternion rotation;
        Math::Vec3 scale;
        float totalWeight = 0.0f;
        bool used = false;
    };

    struct NodeCache
    {
        Node3D* node = nullptr;
        Math::Vec3 position;
        Math::Quaternion rotation;
        Math::Vec3 scale;
        float totalWeight = 0.0f;
        std::size_t weightPass = 0;
        bool positionUsed = false;
        bool rotationUsed = false;
        bool scaleUsed = false;
    };

    struct NodeTrackRef
    {
        const NodeTrack* track = nullptr;
        std::size_t cache = 0;
    };

    const Entry* findEntry(const ct::String& name) const;
    double blendTimeFor(const ct::String& from, const ct::String& to, double custom) const;
    float currentBlendAmount() const;
    void processAnimation(double delta);
    void processAnimationInternal(double delta);
    void applyValueInstances(Node* root);
    void collectMethodCalls(Node* root);
    void applyAnimationTracks(Node* root);
    void processPlaybackData(PlaybackData& data, double delta, float blend, bool started,
            bool isCurrent);
    void blendPlaybackData(double delta, bool started);
    void applyInstances(Skeleton3D& skeleton);
    void applyNodeInstances();
    NodeCache* nodeCacheFor(const NodeTrack& track);
    void stopInternal(bool reset, bool keepState);
    void loadAnimationFiles();
    void dropFileAnimations();
    void seekInternal(double time, bool update);
    void setProcessing(bool enable);

    ct::Vector<AnimationClip*> mOwnedClips;
    ct::Vector<Entry> mAnimations;
    ct::Vector<BlendTime> mBlendTimes;
    ct::Vector<NextEntry> mNext;
    ct::String mSkeletonPath;
    ct::String mRootNode;
    ct::String mAutoplay;
    ct::String mAnimationFiles;
    PlaybackData mCurrent;
    ct::String mAssigned;
    ct::Vector<Blend> mBlends;
    ct::Vector<ct::String> mQueue;
    ct::Vector<Instance> mInstances;
    ct::Vector<TrackCache> mCache;
    ct::Vector<LocalPose> mScratch;
    // The bones some clip of the library animates, for a skeleton of mUsedBonesFor bones: found
    // again only when the library changes (a character carries dozens of clips, and walking all
    // their tracks every frame cost more than posing it).
    ct::Vector<int> mUsedBones;
    std::uint32_t mUsedBonesFor = 0;
    bool mUsedBonesDirty = true;
    ct::Vector<NodeCache> mNodeCache;
    ct::Vector<NodeTrackRef> mNodeTrackRefs;
    ct::Vector<ValueCache> mValueCache;
    ct::Vector<ValueTrackRef> mValueTrackRefs;
    ct::Vector<PendingCall> mPendingCalls;
    ct::Vector<const AnimationTrack*> mPlayingTracks;
    ct::Vector<int> mKeyIndices;
    ct::String mFinished;
    float mSpeedScale;
    double mDefaultBlendTime;
    CallbackModeProcess mCallbackMode;
    bool mActive;
    bool mDeterministic;
    bool mPlaying;
    bool mSeeked;
    bool mStarted;
    bool mEndReached;
    bool mEndNotify;
    bool mProcessing;
    bool mImportingFile;
};

} // namespace prisma3d
