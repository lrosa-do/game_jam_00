#include "prisma3d/scene/AnimationPlayer.h"

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/Skeleton3D.h"

#include "platform.h"

#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

const double kEpsilon = 1e-5;
const ct::String kEmpty;

bool isZeroApprox(double value)
{
    return fabs(value) < kEpsilon;
}

bool isEqualApprox(double a, double b)
{
    if (a == b) return true;
    double tolerance = kEpsilon * fabs(a);
    if (tolerance < kEpsilon) tolerance = kEpsilon;
    return fabs(a - b) < tolerance;
}

bool isLessApprox(double a, double b)
{
    return a < b && !isEqualApprox(a, b);
}

bool isGreaterApprox(double a, double b)
{
    return a > b && !isEqualApprox(a, b);
}

bool isLessOrEqualApprox(double a, double b)
{
    return a < b || isEqualApprox(a, b);
}

bool isGreaterOrEqualApprox(double a, double b)
{
    return a > b || isEqualApprox(a, b);
}

double fposmod(double value, double length)
{
    double result = fmod(value, length);
    if ((result < 0.0 && length > 0.0) || (result > 0.0 && length < 0.0)) result += length;
    return result;
}

double pingpong(double value, double length)
{
    if (length == 0.0) return 0.0;
    const double span = length * 2.0;
    const double fraction = (value - length) / span;
    return fabs((fraction - floor(fraction)) * span - length);
}

Math::Quaternion interpolateViaRest(const Math::Quaternion& from, const Math::Quaternion& to,
        float weight, const Math::Quaternion& rest)
{
    const Math::Quaternion delta = rest.Inverse() * to;
    return (from * Math::Quaternion::Slerp(Math::Quaternion::Identity(), delta, weight))
            .Normalized();
}

} // namespace

AnimationPlayer::AnimationPlayer()
        : Node(),
          mAnimations(),
          mBlendTimes(),
          mNext(),
          mSkeletonPath(".."),
          mRootNode(".."),
          mAutoplay(),
          mAnimationFiles(),
          mCurrent(),
          mAssigned(),
          mBlends(),
          mQueue(),
          mInstances(),
          mCache(),
          mScratch(),
          mFinished(),
          mSpeedScale(1.0f),
          mDefaultBlendTime(0.0),
          mCallbackMode(CallbackModeProcess::Idle),
          mActive(true),
          mDeterministic(false),
          mPlaying(false),
          mSeeked(false),
          mStarted(false),
          mEndReached(false),
          mEndNotify(false),
          mProcessing(false),
          mImportingFile(false)
{
}

AnimationPlayer::~AnimationPlayer()
{
    for (AnimationClip* clip: mOwnedClips) delete clip;
}

bool AnimationPlayer::addAnimation(const ct::String& name, const AnimationClip* clip)
{
    if (name.empty() || !clip)
    {
        log_error("animation player: an animation needs a name and a clip");
        return false;
    }
    for (Entry& entry: mAnimations)
    {
        if (entry.name == name)
        {
            entry.clip = clip;
            mUsedBonesDirty = true;
            if (mCurrent.enabled && mCurrent.name == name)
            {
                mCurrent.clip = clip;
                mCurrent.length = clip->duration();
            }
            return true;
        }
    }
    Entry entry;
    entry.name = name;
    entry.clip = clip;
    entry.fromFile = mImportingFile;
    mAnimations.push_back(entry);
    mUsedBonesDirty = true;
    notifyPropertyListChanged();
    return true;
}

void AnimationPlayer::validateProperty(PropertyInfo& info) const
{
    Node::validateProperty(info);
    if (info.name != "autoplay" || mAnimations.empty()) return;
    info.hint = PropertyHint::Enum;
    info.hintString = ct::String();
    for (const Entry& entry: mAnimations)
    {
        info.hintString += ',';
        info.hintString += entry.name;
    }
}

namespace
{

ct::String fileStem(const ct::String& path)
{
    std::size_t start = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') start = i + 1;
    std::size_t end = path.size();
    for (std::size_t i = start; i < path.size(); ++i)
        if (path[i] == '.') end = i;
    return path.substr(start, end - start);
}

} // namespace

int AnimationPlayer::importAnimations(const ct::String& path, const ct::String& prefix)
{
    ModelLoader* loader = ModelLoader::singleton();
    if (!loader)
    {
        log_error("animation player: there is no model loader");
        return -1;
    }
    ModelData data;
    if (!loader->load(path, data))
    {
        log_error("animation player: %s", data.error.c_str());
        return -1;
    }
    if (data.animations.empty())
    {
        log_warn("animation player: '%s' has no animations", path.c_str());
        return 0;
    }
    Skeleton3D* skeleton = getSkeleton();
    const ct::String stem = fileStem(path);
    int added = 0;
    for (std::size_t a = 0; a < data.animations.size(); ++a)
    {
        const AnimationClip& source = data.animations[a];
        const Skeleton* sourceSkeleton =
                source.skeleton() >= 0 &&
                                static_cast<std::size_t>(source.skeleton()) < data.skeletons.size()
                        ? &data.skeletons[static_cast<std::size_t>(source.skeleton())]
                        : nullptr;
        AnimationClip* clip = new AnimationClip(source);
        clip->tracks().clear();
        int dropped = 0;
        for (std::size_t t = 0; t < source.tracks().size(); ++t)
        {
            const BoneTrack& track = source.tracks()[t];
            if (!skeleton || !sourceSkeleton || track.bone < 0 ||
                    static_cast<std::uint32_t>(track.bone) >= sourceSkeleton->boneCount())
            {
                ++dropped;
                continue;
            }
            const int bone = skeleton->findBone(
                    sourceSkeleton->bone(static_cast<std::uint32_t>(track.bone)).name.c_str());
            if (bone < 0)
            {
                ++dropped;
                continue;
            }
            BoneTrack copy = track;
            copy.bone = bone;
            clip->tracks().push_back(copy);
        }
        ct::String name = source.name();
        if (name.empty())
        {
            name = stem;
            if (data.animations.size() > 1)
            {
                name += '_';
                name.append_number(static_cast<unsigned long long>(a + 1));
            }
        }
        name = prefix + name;
        if (clip->tracks().empty() && clip->nodeTracks().empty() && clip->methodTracks().empty() &&
                clip->valueTracks().empty())
        {
            log_warn("animation player: '%s' was skipped, no bone of '%s' matches the skeleton",
                    name.c_str(), path.c_str());
            delete clip;
            continue;
        }
        if (dropped > 0)
            log_warn("animation player: '%s' lost %d of %d bone tracks with no bone in the "
                     "skeleton",
                    name.c_str(), dropped, static_cast<int>(source.tracks().size()));
        clip->setSkeleton(0);
        ct::String unique = name;
        for (int suffix = 2; hasAnimation(unique); ++suffix)
        {
            unique = name;
            unique += '_';
            unique.append_number(static_cast<unsigned long long>(suffix));
        }
        mOwnedClips.push_back(clip);
        addAnimation(unique, clip);
        ++added;
    }
    return added;
}

bool AnimationPlayer::addAnimationCopy(const ct::String& name, const AnimationClip& clip)
{
    if (name.empty()) return false;
    AnimationClip* owned = new AnimationClip(clip);
    mOwnedClips.push_back(owned);
    return addAnimation(name, owned);
}

ct::String AnimationPlayer::getAnimationNames() const
{
    ct::String names;
    for (std::size_t i = 0; i < mAnimations.size(); ++i)
    {
        if (i) names += ", ";
        names += mAnimations[i].name;
    }
    return names;
}

void AnimationPlayer::setAnimationSource(const ct::String& path)
{
    if (path.empty()) return;
    ModelLoader* loader = ModelLoader::singleton();
    if (!loader || !loader->importerFor(path)) return;
    importAnimations(path);
}

void AnimationPlayer::setAnimationFiles(const ct::String& files)
{
    if (files == mAnimationFiles) return;
    mAnimationFiles = files;
    if (isInsideTree()) loadAnimationFiles();
}

const ct::String& AnimationPlayer::getAnimationFiles() const
{
    return mAnimationFiles;
}

void AnimationPlayer::loadAnimationFiles()
{
    dropFileAnimations();
    std::size_t begin = 0;
    while (begin <= mAnimationFiles.size())
    {
        std::size_t end = mAnimationFiles.find(';', begin);
        if (end == ct::String::npos) end = mAnimationFiles.size();
        std::size_t first = begin;
        std::size_t last = end;
        while (first < last && mAnimationFiles[first] == ' ') ++first;
        while (last > first && mAnimationFiles[last - 1] == ' ') --last;
        if (last > first)
        {
            mImportingFile = true;
            importAnimations(mAnimationFiles.substr(first, last - first));
            mImportingFile = false;
        }
        begin = end + 1;
    }
}

void AnimationPlayer::dropFileAnimations()
{
    bool changed = false;
    for (std::size_t i = mAnimations.size(); i > 0; --i)
    {
        Entry& entry = mAnimations[i - 1];
        if (!entry.fromFile) continue;
        if (mCurrent.enabled && mCurrent.name == entry.name) stopInternal(true, true);
        for (std::size_t k = 0; k < mOwnedClips.size(); ++k)
        {
            if (mOwnedClips[k] != entry.clip) continue;
            delete mOwnedClips[k];
            mOwnedClips.erase(mOwnedClips.begin() + k);
            break;
        }
        mAnimations.erase(mAnimations.begin() + (i - 1));
        mUsedBonesDirty = true;
        changed = true;
    }
    if (changed) notifyPropertyListChanged();
}

bool AnimationPlayer::removeAnimation(const ct::String& name)
{
    for (std::size_t i = 0; i < mAnimations.size(); ++i)
    {
        if (mAnimations[i].name != name) continue;
        if (mCurrent.enabled && mCurrent.name == name) stopInternal(true, true);
        mAnimations.erase(mAnimations.begin() + i);
        mUsedBonesDirty = true;
        notifyPropertyListChanged();
        return true;
    }
    return false;
}

bool AnimationPlayer::hasAnimation(const ct::String& name) const
{
    return findEntry(name) != nullptr;
}

const AnimationClip* AnimationPlayer::getAnimation(const ct::String& name) const
{
    const Entry* entry = findEntry(name);
    return entry ? entry->clip : nullptr;
}

std::size_t AnimationPlayer::getAnimationCount() const
{
    return mAnimations.size();
}

const ct::String& AnimationPlayer::getAnimationName(std::size_t index) const
{
    return index < mAnimations.size() ? mAnimations[index].name : kEmpty;
}

void AnimationPlayer::setSkeletonPath(const ct::String& path)
{
    mSkeletonPath = path;
}

const ct::String& AnimationPlayer::getSkeletonPath() const
{
    return mSkeletonPath;
}

Skeleton3D* AnimationPlayer::getSkeleton() const
{
    Node* node = getNode(mSkeletonPath);
    return node ? node->castTo<Skeleton3D>() : nullptr;
}

bool AnimationPlayer::setAnimationLoop(const ct::String& animation, bool loop)
{
    const Entry* entry = findEntry(animation);
    if (!entry || !entry->clip)
    {
        log_error("animation player: animation not found: %s", animation.c_str());
        return false;
    }
    const LoopMode mode = loop ? LoopMode::Linear : LoopMode::None;
    if (entry->clip->loopMode() == mode) return true;
    for (std::size_t i = 0; i < mOwnedClips.size(); ++i)
    {
        if (mOwnedClips[i] != entry->clip) continue;
        mOwnedClips[i]->setLoopMode(mode);
        return true;
    }
    AnimationClip* owned = new AnimationClip(*entry->clip);
    owned->setLoopMode(mode);
    mOwnedClips.push_back(owned);
    return addAnimation(animation, owned);
}

bool AnimationPlayer::getAnimationLoop(const ct::String& animation) const
{
    const Entry* entry = findEntry(animation);
    return entry && entry->clip && entry->clip->loopMode() != LoopMode::None;
}

void AnimationPlayer::animationSetNext(const ct::String& animation, const ct::String& next)
{
    if (!hasAnimation(animation))
    {
        log_error("animation player: animation not found: %s", animation.c_str());
        return;
    }
    for (NextEntry& entry: mNext)
    {
        if (entry.from == animation)
        {
            entry.next = next;
            return;
        }
    }
    NextEntry entry;
    entry.from = animation;
    entry.next = next;
    mNext.push_back(entry);
}

const ct::String& AnimationPlayer::animationGetNext(const ct::String& animation) const
{
    for (const NextEntry& entry: mNext)
        if (entry.from == animation) return entry.next;
    return kEmpty;
}

void AnimationPlayer::setBlendTime(const ct::String& from, const ct::String& to, double time)
{
    if (time < 0.0)
    {
        log_error("animation player: blend time cannot be negative");
        return;
    }
    for (std::size_t i = 0; i < mBlendTimes.size(); ++i)
    {
        if (mBlendTimes[i].from != from || mBlendTimes[i].to != to) continue;
        if (time == 0.0) mBlendTimes.erase(mBlendTimes.begin() + i);
        else
            mBlendTimes[i].time = time;
        return;
    }
    if (time == 0.0) return;
    BlendTime entry;
    entry.from = from;
    entry.to = to;
    entry.time = time;
    mBlendTimes.push_back(entry);
}

double AnimationPlayer::getBlendTime(const ct::String& from, const ct::String& to) const
{
    for (const BlendTime& entry: mBlendTimes)
        if (entry.from == from && entry.to == to) return entry.time;
    return 0.0;
}

void AnimationPlayer::setDefaultBlendTime(double time)
{
    mDefaultBlendTime = time;
}

double AnimationPlayer::getDefaultBlendTime() const
{
    return mDefaultBlendTime;
}

void AnimationPlayer::play(const ct::String& name, double customBlend, float customScale,
        bool fromEnd)
{
    const ct::String target = name.empty() ? mAssigned : name;
    const Entry* entry = findEntry(target);
    if (!entry)
    {
        log_error("animation player: animation not found: %s", target.c_str());
        return;
    }

    if (mCurrent.enabled)
    {
        const double blendTime = blendTimeFor(mCurrent.name, target, customBlend);
        if (isGreaterApprox(blendTime, 0.0))
        {
            Blend blend;
            blend.data = mCurrent;
            blend.blendLeft = currentBlendAmount();
            blend.blendTime = blendTime;
            mBlends.push_back(blend);
        }
        else
        {
            mBlends.clear();
        }
    }

    mCurrent.enabled = true;
    mCurrent.name = target;
    mCurrent.clip = entry->clip;
    mCurrent.length = entry->clip->duration();
    mCurrent.speedScale = customScale;
    const double start = 0.0;
    const double end = mCurrent.length;

    if (!mEndReached) mQueue.clear();

    if (mAssigned != target)
    {
        mCurrent.pos = fromEnd ? end : start;
        mAssigned = target;
    }
    else
    {
        if (fromEnd && isLessOrEqualApprox(mCurrent.pos, start)) seekInternal(end, true);
        else if (!fromEnd && isGreaterOrEqualApprox(mCurrent.pos, end))
            seekInternal(start, true);
        else if (mPlaying)
            return;
    }

    mSeeked = false;
    mStarted = true;
    setProcessing(true);
    mPlaying = true;
    if (animationStarted) animationStarted(mAssigned);

    const ct::String& next = animationGetNext(target);
    if (!next.empty() && hasAnimation(next)) queue(next);
}

void AnimationPlayer::playBackwards(const ct::String& name, double customBlend)
{
    play(name, customBlend, -1.0f, true);
}

void AnimationPlayer::queue(const ct::String& name)
{
    if (!hasAnimation(name))
    {
        log_error("animation player: animation not found: %s", name.c_str());
        return;
    }
    if (!isPlaying()) play(name);
    else
        mQueue.push_back(name);
}

std::size_t AnimationPlayer::getQueueCount() const
{
    return mQueue.size();
}

const ct::String& AnimationPlayer::getQueued(std::size_t index) const
{
    return index < mQueue.size() ? mQueue[index] : kEmpty;
}

void AnimationPlayer::clearQueue()
{
    mQueue.clear();
}

void AnimationPlayer::pause()
{
    stopInternal(false, false);
}

void AnimationPlayer::stop(bool keepState)
{
    stopInternal(true, keepState);
}

bool AnimationPlayer::isPlaying() const
{
    return mPlaying;
}

const ct::String& AnimationPlayer::getCurrentAnimation() const
{
    return mPlaying ? mAssigned : kEmpty;
}

const ct::String& AnimationPlayer::getAssignedAnimation() const
{
    return mAssigned;
}

void AnimationPlayer::setSpeedScale(float speed)
{
    mSpeedScale = speed;
}

float AnimationPlayer::getSpeedScale() const
{
    return mSpeedScale;
}

float AnimationPlayer::getPlayingSpeed() const
{
    if (!mPlaying) return 0.0f;
    return mSpeedScale * mCurrent.speedScale;
}

void AnimationPlayer::setAutoplay(const ct::String& name)
{
    mAutoplay = name;
}

const ct::String& AnimationPlayer::getAutoplay() const
{
    return mAutoplay;
}

void AnimationPlayer::seek(double time, bool update)
{
    seekInternal(time, update);
}

double AnimationPlayer::getCurrentAnimationPosition() const
{
    return mCurrent.enabled ? mCurrent.pos : 0.0;
}

double AnimationPlayer::getCurrentAnimationLength() const
{
    return mCurrent.enabled ? mCurrent.length : 0.0;
}

void AnimationPlayer::setActive(bool active)
{
    if (mActive == active) return;
    mActive = active;
    setProcessing(mProcessing);
}

bool AnimationPlayer::isActive() const
{
    return mActive;
}

void AnimationPlayer::setCallbackModeProcess(CallbackModeProcess mode)
{
    if (mCallbackMode == mode) return;
    mCallbackMode = mode;
    setProcessing(mProcessing);
}

AnimationPlayer::CallbackModeProcess AnimationPlayer::getCallbackModeProcess() const
{
    return mCallbackMode;
}

void AnimationPlayer::setRootNode(const ct::String& path)
{
    mRootNode = path;
}

const ct::String& AnimationPlayer::getRootNode() const
{
    return mRootNode;
}

int AnimationPlayer::recordKey(AnimationClip& clip, const Node3D* node, float time, bool position,
        bool rotation, bool scale) const
{
    const Node* root = mRootNode.empty() ? nullptr : getNode(mRootNode);
    if (!root || !node) return -1;

    ct::String path;
    if (node == root) path = ".";
    else
    {
        if (!root->isAncestorOf(node)) return -1;
        path = node->getName();
        for (const Node* parent = node->getParent(); parent && parent != root;
                parent = parent->getParent())
            path = parent->getName() + "/" + path;
    }

    LocalPose pose;
    pose.position = node->getPosition();
    pose.rotation = node->getQuaternion();
    pose.scale = node->getScale();
    return clip.insertNodeKey(path, time, pose, position, rotation, scale);
}

void AnimationPlayer::setDeterministic(bool deterministic)
{
    mDeterministic = deterministic;
}

bool AnimationPlayer::isDeterministic() const
{
    return mDeterministic;
}

void AnimationPlayer::advance(double delta)
{
    processAnimation(delta);
}

void AnimationPlayer::notificationHandler(int what)
{
    Node::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            loadAnimationFiles();
            break;
        case kNotificationReady:
            if (!mAutoplay.empty() && hasAnimation(mAutoplay))
            {
                play(mAutoplay);
                if (mStarted) processAnimation(0.0);
            }
            break;
        case kNotificationProcess:
            if (mCallbackMode == CallbackModeProcess::Idle && mProcessing && mActive)
                processAnimation(getProcessDeltaTime());
            break;
        case kNotificationPhysicsProcess:
            if (mCallbackMode == CallbackModeProcess::Physics && mProcessing && mActive)
                processAnimation(getPhysicsProcessDeltaTime());
            break;
        default:
            break;
    }
}

const AnimationPlayer::Entry* AnimationPlayer::findEntry(const ct::String& name) const
{
    for (const Entry& entry: mAnimations)
        if (entry.name == name) return &entry;
    return nullptr;
}

double AnimationPlayer::blendTimeFor(const ct::String& from, const ct::String& to,
        double custom) const
{
    double blendTime = 0.0;
    if (isGreaterOrEqualApprox(custom, 0.0))
    {
        blendTime = custom;
    }
    else
    {
        const ct::String any("*");
        blendTime = getBlendTime(from, to);
        if (blendTime == 0.0) blendTime = getBlendTime(any, to);
        if (blendTime == 0.0) blendTime = getBlendTime(from, any);
    }
    if (isLessApprox(custom, 0.0) && isZeroApprox(blendTime) && mDefaultBlendTime != 0.0)
        blendTime = mDefaultBlendTime;
    return blendTime;
}

float AnimationPlayer::currentBlendAmount() const
{
    double blend = 1.0;
    for (const Blend& entry: mBlends) blend -= entry.blendLeft;
    return blend > 0.0 ? static_cast<float>(blend) : 0.0f;
}

void AnimationPlayer::processAnimation(double delta)
{
    processAnimationInternal(delta);

    if (mPendingCalls.empty()) return;
    ct::Vector<PendingCall> calls = static_cast<ct::Vector<PendingCall>&&>(mPendingCalls);
    mPendingCalls.clear();
    if (!methodCalled) return;
    for (std::size_t i = 0; i < calls.size(); ++i) methodCalled(calls[i].node, calls[i].method);
}

void AnimationPlayer::processAnimationInternal(double delta)
{
    mInstances.clear();
    if (!mActive) return;
    if (!mCurrent.enabled || !mCurrent.clip)
    {
        setProcessing(false);
        return;
    }

    mEndReached = false;
    mEndNotify = false;
    mFinished.clear();

    const bool started = mStarted;
    mStarted = false;

    blendPlaybackData(delta, started);

    // A hidden skeleton keeps its clock and its queue but is not posed: nobody sees it.
    Skeleton3D* skeleton = getSkeleton();
    if (skeleton && skeleton->isVisibleInTree()) applyInstances(*skeleton);
    applyNodeInstances();
    if (Node* root = mRootNode.empty() ? nullptr : getNode(mRootNode))
    {
        applyValueInstances(root);
        collectMethodCalls(root);
        applyAnimationTracks(root);
    }
    mInstances.clear();

    if (!mEndReached) return;

    if (!mQueue.empty())
    {
        if (!mFinished.empty() && animationFinished) animationFinished(mFinished);
        if (mQueue.empty())
        {
            mEndReached = false;
            mEndNotify = false;
            return;
        }
        const ct::String old = mAssigned;
        const ct::String next = mQueue.front();
        mQueue.erase(mQueue.begin());
        play(next);
        if (mEndNotify && animationChanged) animationChanged(old, mAssigned);
    }
    else
    {
        mPlaying = false;
        setProcessing(false);
        if (mEndNotify && !mFinished.empty() && animationFinished) animationFinished(mFinished);
    }
    mEndReached = false;
    mEndNotify = false;
}

void AnimationPlayer::processPlaybackData(PlaybackData& data, double delta, float blend,
        bool started, bool isCurrent)
{
    if (!data.clip) return;
    const double speed = static_cast<double>(mSpeedScale) * data.speedScale;
    const bool backwards = signbit(speed);
    double step = started ? 0.0 : delta * speed;
    double next = data.pos + step;

    const double start = 0.0;
    const double end = data.length;
    const double span = end - start;
    LoopedFlag loopedFlag = LoopedFlag::None;

    switch (data.clip->loopMode())
    {
        case LoopMode::None:
            if (isLessApprox(next, start)) next = start;
            else if (isGreaterApprox(next, end))
                next = end;
            step = next - data.pos;
            break;
        case LoopMode::Linear:
            if (isLessApprox(next, start) && isGreaterOrEqualApprox(data.pos, start))
                loopedFlag = LoopedFlag::Start;
            if (isGreaterApprox(next, end) && isLessOrEqualApprox(data.pos, end))
                loopedFlag = LoopedFlag::End;
            next = span > 0.0 ? fposmod(next - start, span) + start : start;
            break;
        case LoopMode::PingPong:
            if (isLessApprox(next, start) && isGreaterOrEqualApprox(data.pos, start))
            {
                data.speedScale *= -1.0f;
                loopedFlag = LoopedFlag::Start;
            }
            if (isGreaterApprox(next, end) && isLessOrEqualApprox(data.pos, end))
            {
                data.speedScale *= -1.0f;
                loopedFlag = LoopedFlag::End;
            }
            next = span > 0.0 ? pingpong(next - start, span) + start : start;
            break;
    }

    const double previous = data.pos;

    if (isCurrent && data.clip->loopMode() == LoopMode::None)
    {
        if (!backwards && isLessOrEqualApprox(previous, end) && isEqualApprox(next, end))
        {
            next = end;
            mEndReached = true;
            mEndNotify = isLessApprox(previous, end);
            blend = 1.0f;
        }
        if (backwards && isGreaterOrEqualApprox(previous, start) && isEqualApprox(next, start))
        {
            next = start;
            mEndReached = true;
            mEndNotify = isGreaterApprox(previous, start);
            blend = 1.0f;
        }
    }

    data.pos = next;

    Instance instance;
    instance.clip = data.clip;
    instance.time = started ? previous : next;
    instance.delta = started ? 0.0 : step;
    instance.weight = blend;
    instance.loopedFlag = loopedFlag;
    instance.seeked = started || mSeeked;
    instance.started = started;
    mInstances.push_back(instance);
}

void AnimationPlayer::blendPlaybackData(double delta, bool started)
{
    if (!isZeroApprox(delta)) mSeeked = false;

    processPlaybackData(mCurrent, delta, currentBlendAmount(), started, true);

    if (mEndReached)
    {
        mBlends.clear();
        if (mEndNotify) mFinished = mAssigned;
        return;
    }

    for (std::size_t i = 0; i < mBlends.size();)
    {
        Blend& blend = mBlends[i];
        bool done = false;
        blend.blendLeft -= fabs(static_cast<double>(mSpeedScale) * delta) / blend.blendTime;
        if (blend.blendLeft < 0.0) blend.blendLeft = 0.0;
        if (isLessOrEqualApprox(blend.blendLeft, 0.0))
        {
            done = true;
            blend.blendLeft = kEpsilon;
        }
        processPlaybackData(blend.data, delta, static_cast<float>(blend.blendLeft), false, false);
        if (done) mBlends.erase(mBlends.begin() + i);
        else
            ++i;
    }
}

void AnimationPlayer::applyInstances(Skeleton3D& skeleton)
{
    const std::uint32_t bones = skeleton.getBoneCount();
    if (bones == 0) return;

    if (mUsedBonesDirty || mUsedBonesFor != bones)
    {
        ct::Vector<bool> seen;
        seen.resize(bones, false);
        mUsedBones.clear();
        for (const Entry& entry: mAnimations)
        {
            if (!entry.clip) continue;
            for (const BoneTrack& track: entry.clip->tracks())
            {
                if (track.bone < 0 || static_cast<std::uint32_t>(track.bone) >= bones) continue;
                if (seen[static_cast<std::size_t>(track.bone)]) continue;
                seen[static_cast<std::size_t>(track.bone)] = true;
                mUsedBones.push_back(track.bone);
            }
        }
        mUsedBonesFor = bones;
        mUsedBonesDirty = false;
    }

    mCache.resize(bones);
    for (std::uint32_t b = 0; b < bones; ++b)
    {
        mCache[b].used = false;
        mCache[b].totalWeight = 0.0f;
    }
    for (int bone: mUsedBones)
    {
        TrackCache& cache = mCache[static_cast<std::size_t>(bone)];
        cache.used = true;
        cache.init = skeleton.getBoneRestPose(bone);
        cache.position = cache.init.position;
        cache.rotation = cache.init.rotation;
        cache.scale = cache.init.scale;
    }

    for (const Instance& instance: mInstances)
    {
        for (const BoneTrack& track: instance.clip->tracks())
        {
            if (track.bone < 0 || static_cast<std::uint32_t>(track.bone) >= bones) continue;
            mCache[static_cast<std::size_t>(track.bone)].totalWeight += instance.weight;
        }
    }

    mScratch.resize(bones);
    for (const Instance& instance: mInstances)
    {
        instance.clip->sample(static_cast<float>(instance.time), mScratch);
        for (const BoneTrack& track: instance.clip->tracks())
        {
            if (track.bone < 0 || static_cast<std::uint32_t>(track.bone) >= bones) continue;
            TrackCache& cache = mCache[static_cast<std::size_t>(track.bone)];
            float blend = instance.weight;
            if (!mDeterministic)
            {
                if (isZeroApprox(cache.totalWeight)) continue;
                blend = blend / cache.totalWeight;
            }
            if (isZeroApprox(blend)) continue;
            const LocalPose& key = mScratch[static_cast<std::size_t>(track.bone)];
            cache.position += (key.position - cache.init.position) * blend;
            cache.rotation =
                    interpolateViaRest(cache.rotation, key.rotation, blend, cache.init.rotation);
            cache.scale += (key.scale - cache.init.scale) * blend;
        }
    }

    for (std::uint32_t b = 0; b < bones; ++b)
    {
        const TrackCache& cache = mCache[b];
        if (!cache.used || (!mDeterministic && isZeroApprox(cache.totalWeight))) continue;
        skeleton.setBonePosePosition(static_cast<int>(b), cache.position);
        skeleton.setBonePoseRotation(static_cast<int>(b), cache.rotation);
        skeleton.setBonePoseScale(static_cast<int>(b), cache.scale);
    }
}

AnimationPlayer::NodeCache* AnimationPlayer::nodeCacheFor(const NodeTrack& track)
{
    for (const NodeTrackRef& ref: mNodeTrackRefs)
        if (ref.track == &track) return &mNodeCache[ref.cache];
    return nullptr;
}

void AnimationPlayer::applyNodeInstances()
{
    mNodeCache.clear();
    mNodeTrackRefs.clear();

    Node* root = mRootNode.empty() ? nullptr : getNode(mRootNode);
    if (!root) return;

    for (const Entry& entry: mAnimations)
    {
        if (!entry.clip) continue;
        for (const NodeTrack& track: entry.clip->nodeTracks())
        {
            Node* target = root->getNode(track.path);
            Node3D* node = target ? target->castTo<Node3D>() : nullptr;
            if (!node) continue;

            std::size_t index = mNodeCache.size();
            for (std::size_t i = 0; i < mNodeCache.size(); ++i)
            {
                if (mNodeCache[i].node != node) continue;
                index = i;
                break;
            }
            if (index == mNodeCache.size())
            {
                NodeCache cache;
                cache.node = node;
                cache.position = Math::Vec3(0.0f, 0.0f, 0.0f);
                cache.rotation = Math::Quaternion::Identity();
                cache.scale = Math::Vec3(1.0f, 1.0f, 1.0f);
                mNodeCache.push_back(cache);
            }
            NodeCache& cache = mNodeCache[index];
            if (!track.positionTimes.empty()) cache.positionUsed = true;
            if (!track.rotationTimes.empty()) cache.rotationUsed = true;
            if (!track.scaleTimes.empty()) cache.scaleUsed = true;

            NodeTrackRef ref;
            ref.track = &track;
            ref.cache = index;
            mNodeTrackRefs.push_back(ref);
        }
    }
    if (mNodeCache.empty()) return;

    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        const Instance& instance = mInstances[i];
        for (const NodeTrack& track: instance.clip->nodeTracks())
        {
            NodeCache* cache = nodeCacheFor(track);
            if (!cache || cache->weightPass == i + 1) continue;
            cache->weightPass = i + 1;
            cache->totalWeight += instance.weight;
        }
    }

    const Math::Vec3 one(1.0f, 1.0f, 1.0f);
    for (const Instance& instance: mInstances)
    {
        const float time = static_cast<float>(instance.time);
        for (const NodeTrack& track: instance.clip->nodeTracks())
        {
            NodeCache* cache = nodeCacheFor(track);
            if (!cache) continue;
            float blend = instance.weight;
            if (!mDeterministic)
            {
                if (isZeroApprox(cache->totalWeight)) continue;
                blend = blend / cache->totalWeight;
            }
            if (isZeroApprox(blend)) continue;

            Math::Vec3 position;
            if (track.samplePosition(time, position)) cache->position += position * blend;
            Math::Quaternion rotation;
            if (track.sampleRotation(time, rotation))
                cache->rotation = interpolateViaRest(cache->rotation, rotation, blend,
                        Math::Quaternion::Identity());
            Math::Vec3 scale;
            if (track.sampleScale(time, scale)) cache->scale += (scale - one) * blend;
        }
    }

    for (const NodeCache& cache: mNodeCache)
    {
        if (!mDeterministic && isZeroApprox(cache.totalWeight)) continue;
        if (cache.positionUsed) cache.node->setPosition(cache.position);
        if (cache.rotationUsed) cache.node->setQuaternion(cache.rotation);
        if (cache.scaleUsed) cache.node->setScale(cache.scale);
    }
}

void AnimationPlayer::applyValueInstances(Node* root)
{
    mValueCache.clear();
    mValueTrackRefs.clear();

    for (const Entry& entry: mAnimations)
    {
        if (!entry.clip) continue;
        for (const ValueTrack& track: entry.clip->valueTracks())
        {
            if (track.times.empty() || track.values.empty()) continue;
            Node* node = root->getNode(track.path);
            if (!node) continue;

            std::size_t index = mValueCache.size();
            for (std::size_t i = 0; i < mValueCache.size(); ++i)
            {
                if (mValueCache[i].node != node || mValueCache[i].first->property != track.property)
                    continue;
                index = i;
                break;
            }
            if (index == mValueCache.size())
            {
                ValueCache cache;
                cache.node = node;
                cache.first = &track;
                cache.init = track.values[0];
                cache.value = cache.init;
                mValueCache.push_back(cache);
            }

            ValueTrackRef ref;
            ref.track = &track;
            ref.cache = index;
            mValueTrackRefs.push_back(ref);
        }
    }
    if (mValueCache.empty()) return;

    auto cacheFor = [this](const ValueTrack& track) -> ValueCache*
    {
        for (const ValueTrackRef& ref: mValueTrackRefs)
            if (ref.track == &track) return &mValueCache[ref.cache];
        return nullptr;
    };

    for (std::size_t i = 0; i < mInstances.size(); ++i)
    {
        const Instance& instance = mInstances[i];
        for (const ValueTrack& track: instance.clip->valueTracks())
        {
            ValueCache* cache = cacheFor(track);
            if (!cache || cache->weightPass == i + 1) continue;
            cache->weightPass = i + 1;
            cache->totalWeight += instance.weight;
        }
    }

    for (const Instance& instance: mInstances)
    {
        for (const ValueTrack& track: instance.clip->valueTracks())
        {
            ValueCache* cache = cacheFor(track);
            if (!cache) continue;
            float blend = instance.weight;
            if (!mDeterministic)
            {
                if (isZeroApprox(cache->totalWeight)) continue;
                blend = blend / cache->totalWeight;
            }
            if (isZeroApprox(blend)) continue;
            float key = 0.0f;
            if (track.sample(static_cast<float>(instance.time), key))
                cache->value += (key - cache->init) * blend;
        }
    }

    if (!valueChanged) return;
    for (std::size_t i = 0; i < mValueCache.size(); ++i)
    {
        const ValueCache cache = mValueCache[i];
        if (!mDeterministic && isZeroApprox(cache.totalWeight)) continue;
        valueChanged(cache.node, cache.first->property, cache.value);
    }
}

void AnimationPlayer::collectMethodCalls(Node* root)
{
    for (const Instance& instance: mInstances)
    {
        if (isZeroApprox(instance.weight)) continue;
        if (instance.seeked && !instance.started) continue;

        auto collect = [this, &instance](Node* node, const ct::Vector<float>& times,
                               const ct::Vector<ct::String>& names)
        {
            mKeyIndices.clear();
            if (instance.started)
            {
                const int index = AnimationClip::findKey(times, instance.time, true);
                if (index >= 0) mKeyIndices.push_back(index);
            }
            else
            {
                instance.clip->keyIndicesInRange(times, instance.time, instance.delta,
                        instance.loopedFlag, mKeyIndices);
            }
            for (std::size_t i = 0; i < mKeyIndices.size(); ++i)
            {
                const std::size_t key = static_cast<std::size_t>(mKeyIndices[i]);
                if (key >= names.size()) continue;
                PendingCall call;
                call.node = node;
                call.method = names[key];
                mPendingCalls.push_back(call);
            }
        };

        for (const MethodTrack& track: instance.clip->methodTracks())
        {
            Node* node = root->getNode(track.path);
            if (node) collect(node, track.times, track.methods);
        }

        const ct::Vector<AnimationEvent>& events = instance.clip->events();
        if (!events.empty())
        {
            ct::Vector<float> times;
            ct::Vector<ct::String> names;
            for (const AnimationEvent& event: events)
            {
                times.push_back(event.time);
                names.push_back(event.name);
            }
            collect(root, times, names);
        }
    }
}

void AnimationPlayer::applyAnimationTracks(Node* root)
{
    const ct::String stop("[stop]");

    auto playingIndex = [this](const AnimationTrack* track) -> std::size_t
    {
        for (std::size_t i = 0; i < mPlayingTracks.size(); ++i)
            if (mPlayingTracks[i] == track) return i;
        return mPlayingTracks.size();
    };

    for (const Instance& instance: mInstances)
    {
        if (isZeroApprox(instance.weight)) continue;
        if (instance.seeked && !instance.started) continue;

        for (const AnimationTrack& track: instance.clip->animationTracks())
        {
            Node* node = root->getNode(track.path);
            AnimationPlayer* other = node ? node->castTo<AnimationPlayer>() : nullptr;
            if (!other || other == this) continue;

            if (instance.started)
            {
                const int index = AnimationClip::findKey(track.times, instance.time, false);
                if (index < 0 || static_cast<std::size_t>(index) >= track.animations.size())
                    continue;
                const ct::String& name = track.animations[static_cast<std::size_t>(index)];
                const AnimationClip* clip = name == stop ? nullptr : other->getAnimation(name);
                if (!clip) continue;

                const double position = track.times[static_cast<std::size_t>(index)];
                const double end = clip->duration();
                const bool backward = signbit(instance.delta);
                double at = 0.0;
                switch (clip->loopMode())
                {
                    case LoopMode::None:
                        if ((!backward && isGreaterOrEqualApprox(instance.time, position + end)) ||
                                (backward && isLessOrEqualApprox(instance.time, position)))
                            continue;
                        at = instance.time - position < end ? instance.time - position : end;
                        break;
                    case LoopMode::Linear:
                        at = end > 0.0 ? fposmod(instance.time - position, end) : 0.0;
                        break;
                    case LoopMode::PingPong:
                        at = end > 0.0 ? pingpong(instance.time - position, end) : 0.0;
                        break;
                }
                other->play(name);
                other->seek(at);
                if (playingIndex(&track) == mPlayingTracks.size()) mPlayingTracks.push_back(&track);
                continue;
            }

            mKeyIndices.clear();
            instance.clip->keyIndicesInRange(track.times, instance.time, instance.delta,
                    instance.loopedFlag, mKeyIndices);
            if (mKeyIndices.empty()) continue;
            const std::size_t key = static_cast<std::size_t>(mKeyIndices.back());
            if (key >= track.animations.size()) continue;
            const ct::String& name = track.animations[key];
            const std::size_t playing = playingIndex(&track);
            if (name == stop || !other->hasAnimation(name))
            {
                if (playing != mPlayingTracks.size())
                {
                    mPlayingTracks.erase(mPlayingTracks.begin() + playing);
                    other->stop();
                }
            }
            else
            {
                other->play(name);
                if (playing == mPlayingTracks.size()) mPlayingTracks.push_back(&track);
            }
        }
    }
}

void AnimationPlayer::stopInternal(bool reset, bool keepState)
{
    const double start = 0.0;
    if (reset)
    {
        mBlends.clear();
        if (keepState) mCurrent.pos = start;
        else
            seekInternal(start, true);
        mCurrent.enabled = false;
        mCurrent.name.clear();
        mCurrent.clip = nullptr;
        mCurrent.speedScale = 1.0f;
    }
    setProcessing(false);
    mQueue.clear();
    mPlaying = false;
}

void AnimationPlayer::seekInternal(double time, bool update)
{
    if (!mActive) return;

    if (mStarted) processAnimation(0.0);

    mCurrent.pos = time;
    if (!mCurrent.enabled)
    {
        if (!mAssigned.empty())
        {
            const Entry* entry = findEntry(mAssigned);
            if (!entry)
            {
                log_error("animation player: animation not found: %s", mAssigned.c_str());
                return;
            }
            mCurrent.enabled = true;
            mCurrent.name = mAssigned;
            mCurrent.clip = entry->clip;
            mCurrent.length = entry->clip->duration();
        }
        if (!mCurrent.enabled) return;
    }

    const double start = 0.0;
    const double end = mCurrent.length;
    if (time < start) time = start;
    if (time > end) time = end;
    mCurrent.pos = time;
    mSeeked = true;

    if (update)
    {
        processAnimation(0.0);
        mSeeked = false;
    }
}

void AnimationPlayer::setProcessing(bool enable)
{
    mProcessing = enable;
    const bool run = enable && mActive;
    setProcess(run && mCallbackMode == CallbackModeProcess::Idle);
    setPhysicsProcess(run && mCallbackMode == CallbackModeProcess::Physics);
}

void AnimationPlayer::bindProperties()
{
    using DB = NodeClassDB;
    using P = AnimationPlayer;
    DB::addProperty<P>(PropertyInfo::nodePath("root_node"), &P::setRootNode, &P::getRootNode);
    DB::addProperty<P>(PropertyInfo::nodePath("skeleton", "Skeleton3D"), &P::setSkeletonPath,
            &P::getSkeletonPath);
    PropertyInfo libraryInfo = PropertyInfo::resource("animations", "AnimationLibrary");
    libraryInfo.usage = PropertyUsage::Editor;
    DB::addProperty<P>(libraryInfo, &P::setAnimationSource, &P::getAnimationNames);
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "animation_files"), &P::setAnimationFiles,
            &P::getAnimationFiles);
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "autoplay"), &P::setAutoplay,
            &P::getAutoplay);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "speed_scale", -64.0, 64.0, 0.01, true, true),
            &P::setSpeedScale, &P::getSpeedScale);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "playback_default_blend_time", 0.0,
                               3600.0, 0.001, true),
            &P::setDefaultBlendTime, &P::getDefaultBlendTime);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "active"), &P::setActive, &P::isActive);
    DB::addProperty<P>(PropertyInfo::enumeration("callback_mode_process", "Physics,Idle,Manual"),
            &P::setCallbackModeProcess, &P::getCallbackModeProcess);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "deterministic"), &P::setDeterministic,
            &P::isDeterministic);
}

namespace
{

constexpr std::int64_t kAnimationPlayerDataVersion = 1;

}

bool AnimationPlayer::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    bool stored = false;
    for (const Entry& entry: mAnimations)
        if (!entry.fromFile) stored = true;
    if (!stored && mBlendTimes.empty() && mNext.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kAnimationPlayerDataVersion));
    ct::Json clips = ct::Json::array();
    for (const Entry& entry: mAnimations)
    {
        if (entry.fromFile) continue;
        ct::Json item = ct::Json::object();
        item["name"] = ct::Json(entry.name);
        item["clip"] = scenejson::fromClip(*entry.clip);
        clips.push_back(item);
    }
    if (clips.size() > 0) data["animations"] = clips;

    ct::Json blends = ct::Json::array();
    for (const BlendTime& blend: mBlendTimes)
    {
        ct::Json item = ct::Json::object();
        item["from"] = ct::Json(blend.from);
        item["to"] = ct::Json(blend.to);
        item["time"] = scenejson::fromFloat(blend.time);
        blends.push_back(item);
    }
    if (blends.size() > 0) data["blend_times"] = blends;

    ct::Json next = ct::Json::array();
    for (const NextEntry& entry: mNext)
    {
        ct::Json item = ct::Json::object();
        item["from"] = ct::Json(entry.from);
        item["next"] = ct::Json(entry.next);
        next.push_back(item);
    }
    if (next.size() > 0) data["next"] = next;
    return true;
}

bool AnimationPlayer::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kAnimationPlayerDataVersion, result, path)) return false;

    if (const ct::Json* clips = scenejson::arrayField(data, "animations", result, path))
    {
        for (std::size_t i = 0; i < clips->size(); ++i)
        {
            const ct::Json& item = clips->at(i);
            const ct::String itemPath = scenejson::element(path, "animations", i);
            const ct::String name = item.is_object()
                                            ? scenejson::getString(item, "name", ct::String())
                                            : ct::String();
            const ct::Json* clipField = item.is_object() ? item.find("clip") : nullptr;
            if (name.empty() || !clipField)
            {
                result.addError(itemPath, "expected a name and a clip");
                continue;
            }
            AnimationClip clip;
            if (!scenejson::toClip(*clipField, clip, result, scenejson::field(itemPath, "clip")))
                continue;
            AnimationClip* owned = new AnimationClip(clip);
            mOwnedClips.push_back(owned);
            addAnimation(name, owned);
        }
    }
    if (const ct::Json* blends = scenejson::arrayField(data, "blend_times", result, path))
    {
        for (std::size_t i = 0; i < blends->size(); ++i)
        {
            const ct::Json& item = blends->at(i);
            if (!item.is_object())
            {
                result.addError(scenejson::element(path, "blend_times", i), "expected an object");
                continue;
            }
            setBlendTime(scenejson::getString(item, "from", ct::String()),
                    scenejson::getString(item, "to", ct::String()),
                    scenejson::getDouble(item, "time", 0.0));
        }
    }
    if (const ct::Json* next = scenejson::arrayField(data, "next", result, path))
    {
        for (std::size_t i = 0; i < next->size(); ++i)
        {
            const ct::Json& item = next->at(i);
            const ct::String from = item.is_object()
                                            ? scenejson::getString(item, "from", ct::String())
                                            : ct::String();
            if (!hasAnimation(from))
            {
                result.addError(scenejson::element(path, "next", i),
                        "the animation is not in the player");
                continue;
            }
            animationSetNext(from, scenejson::getString(item, "next", ct::String()));
        }
    }
    return true;
}

} // namespace prisma3d
