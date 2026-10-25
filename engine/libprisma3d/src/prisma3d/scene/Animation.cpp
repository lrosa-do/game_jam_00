#include "prisma3d/scene/Animation.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kEpsilon = 1e-8f;
const float kKeyTolerance = 0.0001f;
const float kSpanEpsilon = 0.000001f;
const float kAffineTolerance = 1e-4f;
const int kMaxBones = 65535;

bool finite(float value)
{
    return value - value == 0.0f;
}

bool finite(const Math::Vec4& value)
{
    return finite(value.x) && finite(value.y) && finite(value.z) && finite(value.w);
}

float clamp01(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

std::size_t lowerBound(const ct::Vector<float>& times, std::size_t count, float time)
{
    std::size_t first = 0;
    std::size_t length = count;
    while (length > 0)
    {
        const std::size_t half = length / 2;
        if (times[first + half] < time)
        {
            first += half + 1;
            length -= half + 1;
        }
        else
            length = half;
    }
    return first;
}

std::size_t upperBound(const ct::Vector<float>& times, std::size_t count, float time)
{
    std::size_t first = 0;
    std::size_t length = count;
    while (length > 0)
    {
        const std::size_t half = length / 2;
        if (!(time < times[first + half]))
        {
            first += half + 1;
            length -= half + 1;
        }
        else
            length = half;
    }
    return first;
}

float absolute(float value)
{
    return value < 0.0f ? -value : value;
}

BoneTrack* findTrack(ct::Vector<BoneTrack>& tracks, int bone)
{
    for (std::size_t i = 0; i < tracks.size(); ++i)
    {
        if (tracks[i].bone == bone) return &tracks[i];
    }
    return nullptr;
}

float mixKeys(float from, float to, float amount)
{
    return from + (to - from) * amount;
}

Math::Vec3 mixKeys(const Math::Vec3& from, const Math::Vec3& to, float amount)
{
    return Math::Vec3::Lerp(from, to, amount);
}

Math::Quaternion mixKeys(const Math::Quaternion& from, const Math::Quaternion& to, float amount)
{
    return Math::Quaternion::Lerp(from, to, amount);
}

template<typename T>
bool sampleKeys(const ct::Vector<float>& times, const ct::Vector<T>& values, float time, T& out)
{
    std::size_t count = times.size();
    if (values.size() < count) count = values.size();
    if (count == 0) return false;
    if (count == 1)
    {
        out = values[0];
        return true;
    }

    std::size_t next = upperBound(times, count, time);
    if (next == 0) next = 1;
    if (next >= count) next = count - 1;
    const std::size_t previous = next - 1;
    const float span = times[next] - times[previous];
    const float amount = span > kSpanEpsilon ? clamp01((time - times[previous]) / span) : 0.0f;
    out = mixKeys(values[previous], values[next], amount);
    return true;
}

bool nearTime(double a, double b)
{
    const double difference = a < b ? b - a : a - b;
    return difference < 0.00001;
}

double wrapTime(double value, double length)
{
    if (length <= 0.0) return 0.0;
    double result = fmod(value, length);
    if (result < 0.0) result += length;
    return result;
}

double bounceTime(double value, double length)
{
    if (length <= 0.0) return 0.0;
    const double fraction = (value - length) / (length * 2.0);
    const double wrapped = (fraction - floor(fraction)) * length * 2.0 - length;
    return wrapped < 0.0 ? -wrapped : wrapped;
}

void keysBetween(const ct::Vector<float>& times, double fromTime, double toTime,
        ct::Vector<int>& indices, bool backward)
{
    const int length = static_cast<int>(times.size());
    if (length == 0) return;

    int from = 0;
    int to = length - 1;

    if (!backward)
    {
        while (times[from] < fromTime || nearTime(times[from], fromTime))
        {
            from++;
            if (to < from) return;
        }
        while (times[to] > toTime && !nearTime(times[to], toTime))
        {
            to--;
            if (to < from) return;
        }
    }
    else
    {
        while (times[from] < fromTime && !nearTime(times[from], fromTime))
        {
            from++;
            if (to < from) return;
        }
        while (times[to] > toTime || nearTime(times[to], toTime))
        {
            to--;
            if (to < from) return;
        }
    }

    if (from == to)
    {
        indices.push_back(from);
        return;
    }

    if (!backward)
    {
        for (int i = from; i <= to; i++) indices.push_back(i);
    }
    else
    {
        for (int i = to; i >= from; i--) indices.push_back(i);
    }
}

template<typename T>
void extractKeys(const ct::Vector<float>& times, const ct::Vector<T>& values, float first,
        float last, ct::Vector<float>& outTimes, ct::Vector<T>& outValues)
{
    std::size_t count = times.size();
    if (values.size() < count) count = values.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        if (times[i] < first || times[i] > last) continue;
        outTimes.push_back(times[i] - first);
        outValues.push_back(values[i]);
    }
}

template<typename T>
int insertKey(ct::Vector<float>& times, ct::Vector<T>& values, float time, const T& value)
{
    const std::size_t index = lowerBound(times, times.size(), time);
    if (index < times.size() && absolute(times[index] - time) < kKeyTolerance)
    {
        values[index] = value;
        return static_cast<int>(index);
    }
    if (index > 0 && absolute(times[index - 1] - time) < kKeyTolerance)
    {
        values[index - 1] = value;
        return static_cast<int>(index - 1);
    }
    times.insert(times.begin() + index, time);
    values.insert(values.begin() + index, value);
    return static_cast<int>(index);
}

} // namespace

Math::Mat4 LocalPose::toMatrix() const
{
    const Math::Mat3 basis = rotation.ToMat3();
    const Math::Mat3 scaled(basis.col0 * scale.x, basis.col1 * scale.y, basis.col2 * scale.z);
    return Math::Mat4(scaled, position);
}

bool LocalPose::fromMatrix(const Math::Mat4& matrix, LocalPose& out)
{
    if (!finite(matrix.col0) || !finite(matrix.col1) || !finite(matrix.col2) ||
            !finite(matrix.col3))
        return false;
    if (absolute(matrix.col0.w) > kAffineTolerance || absolute(matrix.col1.w) > kAffineTolerance ||
            absolute(matrix.col2.w) > kAffineTolerance ||
            absolute(matrix.col3.w - 1.0f) > kAffineTolerance)
        return false;

    const Math::Vec3 x = matrix.col0.xyz();
    const Math::Vec3 y = matrix.col1.xyz();
    const Math::Vec3 z = matrix.col2.xyz();

    Math::Vec3 scale(x.Length(), y.Length(), z.Length());
    if (Math::Mat3(x, y, z).Determinant() < 0.0f) scale.x = -scale.x;

    Math::Mat3 basis;
    basis.col0 = scale.x * scale.x > kEpsilon ? x / scale.x : Math::Vec3::UnitX;
    basis.col1 = scale.y * scale.y > kEpsilon ? y / scale.y : Math::Vec3::UnitY;
    basis.col2 = scale.z * scale.z > kEpsilon ? z / scale.z : Math::Vec3::UnitZ;

    out.position = matrix.GetTranslation();
    out.rotation = Math::Quaternion::FromMat3(basis).Normalized();
    out.scale = scale;
    return true;
}

bool Skeleton::empty() const
{
    return mBones.empty();
}

std::uint32_t Skeleton::boneCount() const
{
    return static_cast<std::uint32_t>(mBones.size());
}

const Bone& Skeleton::bone(std::uint32_t index) const
{
    return mBones[index];
}

int Skeleton::findBone(const char* name) const
{
    if (!name) return -1;
    for (std::size_t i = 0; i < mBones.size(); ++i)
    {
        if (mBones[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

bool Skeleton::addBone(const ct::String& name, int parent, const Math::Mat4& bindLocal,
        const Math::Mat4& inverseBind)
{
    if (name.empty() || parent < -1 || parent >= kMaxBones || findBone(name.c_str()) >= 0)
        return false;
    if (mBones.size() >= static_cast<std::size_t>(kMaxBones)) return false;
    Bone bone;
    bone.name = name;
    bone.parent = parent;
    bone.bindLocal = bindLocal;
    bone.inverseBind = inverseBind;
    mBones.push_back(bone);
    mOrder.clear();
    return true;
}

void Skeleton::scalePositions(float scale)
{
    if (scale <= 0.0f) return;
    for (std::size_t i = 0; i < mBones.size(); ++i)
    {
        Bone& bone = mBones[i];
        bone.bindLocal.col3.x *= scale;
        bone.bindLocal.col3.y *= scale;
        bone.bindLocal.col3.z *= scale;
        bone.inverseBind.col3.x *= scale;
        bone.inverseBind.col3.y *= scale;
        bone.inverseBind.col3.z *= scale;
    }
}

bool Skeleton::finalize()
{
    mOrder.clear();
    mOrder.reserve(mBones.size());
    ct::Vector<bool> placed;
    placed.resize(mBones.size(), false);
    bool progress = true;
    while (mOrder.size() < mBones.size() && progress)
    {
        progress = false;
        for (std::size_t i = 0; i < mBones.size(); ++i)
        {
            if (placed[i]) continue;
            const int parent = mBones[i].parent;
            if (parent >= static_cast<int>(mBones.size()) || parent == static_cast<int>(i))
            {
                mOrder.clear();
                return false;
            }
            if (parent < 0 || placed[static_cast<std::size_t>(parent)])
            {
                placed[i] = true;
                mOrder.push_back(static_cast<std::uint16_t>(i));
                progress = true;
            }
        }
    }
    if (mOrder.size() != mBones.size())
    {
        mOrder.clear();
        return false;
    }
    return true;
}

bool Skeleton::finalized() const
{
    return mOrder.size() == mBones.size();
}

const ct::Vector<std::uint16_t>& Skeleton::order() const
{
    return mOrder;
}

void Skeleton::bindPose(ct::Vector<LocalPose>& pose) const
{
    pose.resize(mBones.size());
    for (std::size_t i = 0; i < mBones.size(); ++i)
    {
        if (!LocalPose::fromMatrix(mBones[i].bindLocal, pose[i])) pose[i] = LocalPose();
    }
}

void Skeleton::evaluate(const ct::Vector<LocalPose>& localPose, ct::Vector<Math::Mat4>& globalPose,
        ct::Vector<Math::Mat4>& palette) const
{
    if (localPose.size() != mBones.size() || !finalized()) return;
    globalPose.resize(mBones.size());
    palette.resize(mBones.size());
    for (std::size_t i = 0; i < mOrder.size(); ++i)
    {
        const std::size_t index = mOrder[i];
        const Math::Mat4 local = localPose[index].toMatrix();
        const int parent = mBones[index].parent;
        globalPose[index] =
                parent >= 0 ? globalPose[static_cast<std::size_t>(parent)] * local : local;
        palette[index] = globalPose[index] * mBones[index].inverseBind;
    }
}

const ct::String& AnimationClip::name() const
{
    return mName;
}

void AnimationClip::setName(const ct::String& name)
{
    mName = name;
}

float AnimationClip::duration() const
{
    return mDuration;
}

void AnimationClip::setDuration(float duration)
{
    mDuration = duration > 0.0f ? duration : 0.0f;
}

int AnimationClip::skeleton() const
{
    return mSkeleton;
}

void AnimationClip::setSkeleton(int skeleton)
{
    mSkeleton = skeleton;
}

ct::Vector<BoneTrack>& AnimationClip::tracks()
{
    return mTracks;
}

const ct::Vector<BoneTrack>& AnimationClip::tracks() const
{
    return mTracks;
}

LoopMode AnimationClip::loopMode() const
{
    return mLoopMode;
}

void AnimationClip::setLoopMode(LoopMode mode)
{
    mLoopMode = mode;
}

bool NodeTrack::samplePosition(float time, Math::Vec3& out) const
{
    return sampleKeys(positionTimes, positions, time, out);
}

bool NodeTrack::sampleRotation(float time, Math::Quaternion& out) const
{
    if (!sampleKeys(rotationTimes, rotations, time, out)) return false;
    out = out.Normalized();
    return true;
}

bool NodeTrack::sampleScale(float time, Math::Vec3& out) const
{
    return sampleKeys(scaleTimes, scales, time, out);
}

ct::Vector<NodeTrack>& AnimationClip::nodeTracks()
{
    return mNodeTracks;
}

const ct::Vector<NodeTrack>& AnimationClip::nodeTracks() const
{
    return mNodeTracks;
}

int AnimationClip::addNodeTrack(const ct::String& path)
{
    if (path.empty()) return -1;
    const int existing = findNodeTrack(path);
    if (existing >= 0) return existing;
    NodeTrack track;
    track.path = path;
    mNodeTracks.push_back(track);
    return static_cast<int>(mNodeTracks.size() - 1);
}

int AnimationClip::findNodeTrack(const ct::String& path) const
{
    for (std::size_t i = 0; i < mNodeTracks.size(); ++i)
        if (mNodeTracks[i].path == path) return static_cast<int>(i);
    return -1;
}

int AnimationClip::positionTrackInsertKey(int track, float time, const Math::Vec3& position)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mNodeTracks.size()) return -1;
    NodeTrack& target = mNodeTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.positionTimes, target.positions, time, position);
}

int AnimationClip::rotationTrackInsertKey(int track, float time, const Math::Quaternion& rotation)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mNodeTracks.size()) return -1;
    NodeTrack& target = mNodeTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.rotationTimes, target.rotations, time, rotation.Normalized());
}

int AnimationClip::scaleTrackInsertKey(int track, float time, const Math::Vec3& scale)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mNodeTracks.size()) return -1;
    NodeTrack& target = mNodeTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.scaleTimes, target.scales, time, scale);
}

bool ValueTrack::sample(float time, float& out) const
{
    return sampleKeys(times, values, time, out);
}

ct::Vector<MethodTrack>& AnimationClip::methodTracks()
{
    return mMethodTracks;
}

const ct::Vector<MethodTrack>& AnimationClip::methodTracks() const
{
    return mMethodTracks;
}

int AnimationClip::addMethodTrack(const ct::String& path)
{
    if (path.empty()) return -1;
    for (std::size_t i = 0; i < mMethodTracks.size(); ++i)
        if (mMethodTracks[i].path == path) return static_cast<int>(i);
    MethodTrack track;
    track.path = path;
    mMethodTracks.push_back(track);
    return static_cast<int>(mMethodTracks.size() - 1);
}

int AnimationClip::methodTrackInsertKey(int track, float time, const ct::String& method)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mMethodTracks.size()) return -1;
    MethodTrack& target = mMethodTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.times, target.methods, time, method);
}

ct::Vector<ValueTrack>& AnimationClip::valueTracks()
{
    return mValueTracks;
}

const ct::Vector<ValueTrack>& AnimationClip::valueTracks() const
{
    return mValueTracks;
}

int AnimationClip::addValueTrack(const ct::String& path, const ct::String& property)
{
    if (path.empty() || property.empty()) return -1;
    for (std::size_t i = 0; i < mValueTracks.size(); ++i)
        if (mValueTracks[i].path == path && mValueTracks[i].property == property)
            return static_cast<int>(i);
    ValueTrack track;
    track.path = path;
    track.property = property;
    mValueTracks.push_back(track);
    return static_cast<int>(mValueTracks.size() - 1);
}

int AnimationClip::valueTrackInsertKey(int track, float time, float value)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mValueTracks.size()) return -1;
    ValueTrack& target = mValueTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.times, target.values, time, value);
}

ct::Vector<AnimationTrack>& AnimationClip::animationTracks()
{
    return mAnimationTracks;
}

const ct::Vector<AnimationTrack>& AnimationClip::animationTracks() const
{
    return mAnimationTracks;
}

int AnimationClip::addAnimationTrack(const ct::String& path)
{
    if (path.empty()) return -1;
    for (std::size_t i = 0; i < mAnimationTracks.size(); ++i)
        if (mAnimationTracks[i].path == path) return static_cast<int>(i);
    AnimationTrack track;
    track.path = path;
    mAnimationTracks.push_back(track);
    return static_cast<int>(mAnimationTracks.size() - 1);
}

int AnimationClip::animationTrackInsertKey(int track, float time, const ct::String& animation)
{
    if (track < 0 || static_cast<std::size_t>(track) >= mAnimationTracks.size()) return -1;
    AnimationTrack& target = mAnimationTracks[static_cast<std::size_t>(track)];
    if (time > mDuration) mDuration = time;
    return insertKey(target.times, target.animations, time, animation);
}

void AnimationClip::keyIndicesInRange(const ct::Vector<float>& times, double time, double delta,
        LoopedFlag loopedFlag, ct::Vector<int>& indices) const
{
    if (delta == 0) return;

    double fromTime = time - delta;
    double toTime = time;
    const double start = 0.0;
    const double end = mDuration;

    bool backward = false;
    if (fromTime > toTime)
    {
        backward = true;
        const double swapped = fromTime;
        fromTime = toTime;
        toTime = swapped;
    }

    switch (mLoopMode)
    {
        case LoopMode::None:
            if (fromTime < 0) fromTime = 0;
            if (fromTime > end) fromTime = end;
            if (toTime < 0) toTime = 0;
            if (toTime > end) toTime = end;
            break;
        case LoopMode::Linear:
            if (fromTime > end || fromTime < start) fromTime = wrapTime(fromTime, end);
            if (toTime > end || toTime < start) toTime = wrapTime(toTime, end);

            if (fromTime > toTime)
            {
                const double animEnd = end + 0.00001;
                const double animStart = start - 0.00001;
                if (!backward)
                {
                    keysBetween(times, fromTime, animEnd, indices, backward);
                    keysBetween(times, animStart, toTime, indices, backward);
                }
                else
                {
                    keysBetween(times, animStart, toTime, indices, backward);
                    keysBetween(times, fromTime, animEnd, indices, backward);
                }
                return;
            }

            if (loopedFlag != LoopedFlag::None)
            {
                if (!backward && nearTime(fromTime, start))
                {
                    const int edge = findKey(times, start, true);
                    if (edge >= 0) indices.push_back(edge);
                }
                else if (backward && nearTime(toTime, end))
                {
                    const int edge = findKey(times, end, true);
                    if (edge >= 0) indices.push_back(edge);
                }
            }
            break;
        case LoopMode::PingPong:
            if (fromTime > end || fromTime < start) fromTime = bounceTime(fromTime, end);
            if (toTime > end || toTime < start) toTime = bounceTime(toTime, end);

            if (loopedFlag == LoopedFlag::Start)
            {
                keysBetween(times, start, fromTime, indices, true);
                keysBetween(times, start, toTime, indices, false);
                return;
            }
            if (loopedFlag == LoopedFlag::End)
            {
                keysBetween(times, fromTime, end, indices, false);
                keysBetween(times, toTime, end, indices, true);
                return;
            }
            break;
    }

    keysBetween(times, fromTime, toTime, indices, backward);
}

int AnimationClip::findKey(const ct::Vector<float>& times, double time, bool exact)
{
    int found = -1;
    for (std::size_t i = 0; i < times.size(); ++i)
    {
        if (times[i] > time && !nearTime(times[i], time)) break;
        found = static_cast<int>(i);
    }
    if (found < 0) return -1;
    if (exact && !nearTime(times[static_cast<std::size_t>(found)], time)) return -1;
    return found;
}

int AnimationClip::insertNodeKey(const ct::String& path, float time, const LocalPose& pose,
        bool position, bool rotation, bool scale)
{
    const int track = addNodeTrack(path);
    if (track < 0) return -1;
    if (position) positionTrackInsertKey(track, time, pose.position);
    if (rotation) rotationTrackInsertKey(track, time, pose.rotation);
    if (scale) scaleTrackInsertKey(track, time, pose.scale);
    return track;
}

AnimationClip AnimationClip::extract(float first, float last) const
{
    AnimationClip result;
    result.mName = mName;
    result.mSkeleton = mSkeleton;
    result.mLoopMode = mLoopMode;
    if (last < first) return result;
    result.mDuration = last - first;

    for (const BoneTrack& source: mTracks)
    {
        BoneTrack track;
        track.bone = source.bone;
        std::size_t count = source.times.size();
        if (source.positions.size() < count) count = source.positions.size();
        if (source.rotations.size() < count) count = source.rotations.size();
        if (source.scales.size() < count) count = source.scales.size();
        for (std::size_t i = 0; i < count; ++i)
        {
            if (source.times[i] < first || source.times[i] > last) continue;
            track.times.push_back(source.times[i] - first);
            track.positions.push_back(source.positions[i]);
            track.rotations.push_back(source.rotations[i]);
            track.scales.push_back(source.scales[i]);
        }
        if (!track.times.empty()) result.mTracks.push_back(track);
    }

    for (const NodeTrack& source: mNodeTracks)
    {
        NodeTrack track;
        track.path = source.path;
        extractKeys(source.positionTimes, source.positions, first, last, track.positionTimes,
                track.positions);
        extractKeys(source.rotationTimes, source.rotations, first, last, track.rotationTimes,
                track.rotations);
        extractKeys(source.scaleTimes, source.scales, first, last, track.scaleTimes, track.scales);
        if (!track.positionTimes.empty() || !track.rotationTimes.empty() ||
                !track.scaleTimes.empty())
            result.mNodeTracks.push_back(track);
    }

    for (const MethodTrack& source: mMethodTracks)
    {
        MethodTrack track;
        track.path = source.path;
        extractKeys(source.times, source.methods, first, last, track.times, track.methods);
        if (!track.times.empty()) result.mMethodTracks.push_back(track);
    }

    for (const ValueTrack& source: mValueTracks)
    {
        ValueTrack track;
        track.path = source.path;
        track.property = source.property;
        extractKeys(source.times, source.values, first, last, track.times, track.values);
        if (!track.times.empty()) result.mValueTracks.push_back(track);
    }

    for (const AnimationTrack& source: mAnimationTracks)
    {
        AnimationTrack track;
        track.path = source.path;
        extractKeys(source.times, source.animations, first, last, track.times, track.animations);
        if (!track.times.empty()) result.mAnimationTracks.push_back(track);
    }

    for (const AnimationEvent& event: mEvents)
    {
        if (event.time < first || event.time > last) continue;
        AnimationEvent copy = event;
        copy.time = event.time - first;
        result.mEvents.push_back(copy);
    }
    return result;
}

void AnimationClip::sample(float time, ct::Vector<LocalPose>& pose) const
{
    for (std::size_t t = 0; t < mTracks.size(); ++t)
    {
        const BoneTrack& track = mTracks[t];
        if (track.bone < 0 || static_cast<std::size_t>(track.bone) >= pose.size() ||
                track.times.empty())
            continue;
        std::size_t count = track.times.size();
        if (track.positions.size() < count) count = track.positions.size();
        if (track.rotations.size() < count) count = track.rotations.size();
        if (track.scales.size() < count) count = track.scales.size();
        if (count == 0) continue;

        LocalPose& result = pose[static_cast<std::size_t>(track.bone)];
        if (count == 1)
        {
            result.position = track.positions[0];
            result.rotation = track.rotations[0].Normalized();
            result.scale = track.scales[0];
            continue;
        }

        std::size_t next = upperBound(track.times, count, time);
        if (next == 0) next = 1;
        if (next >= count) next = count - 1;
        const std::size_t previous = next - 1;
        const float span = track.times[next] - track.times[previous];
        const float amount =
                span > kSpanEpsilon ? clamp01((time - track.times[previous]) / span) : 0.0f;
        result.position =
                Math::Vec3::Lerp(track.positions[previous], track.positions[next], amount);
        result.rotation =
                Math::Quaternion::Lerp(track.rotations[previous], track.rotations[next], amount);
        result.scale = Math::Vec3::Lerp(track.scales[previous], track.scales[next], amount);
    }
}

void AnimationClip::setKeyframe(int bone, float time, const LocalPose& pose)
{
    if (bone < 0) return;

    BoneTrack* track = findTrack(mTracks, bone);
    if (!track)
    {
        mTracks.push_back(BoneTrack());
        track = &mTracks.back();
        track->bone = bone;
    }

    const std::size_t index = lowerBound(track->times, track->times.size(), time);
    if (index < track->times.size() && absolute(track->times[index] - time) < kKeyTolerance)
    {
        track->positions[index] = pose.position;
        track->rotations[index] = pose.rotation;
        track->scales[index] = pose.scale;
    }
    else
    {
        track->times.insert(track->times.begin() + index, time);
        track->positions.insert(track->positions.begin() + index, pose.position);
        track->rotations.insert(track->rotations.begin() + index, pose.rotation);
        track->scales.insert(track->scales.begin() + index, pose.scale);
    }
    if (time > mDuration) mDuration = time;
}

void AnimationClip::removeKeyframe(int bone, float time)
{
    BoneTrack* track = findTrack(mTracks, bone);
    if (!track) return;
    const std::size_t index = lowerBound(track->times, track->times.size(), time);
    if (index >= track->times.size() || absolute(track->times[index] - time) >= kKeyTolerance)
        return;
    track->times.erase(track->times.begin() + index);
    track->positions.erase(track->positions.begin() + index);
    track->rotations.erase(track->rotations.begin() + index);
    track->scales.erase(track->scales.begin() + index);
}

void AnimationClip::addEvent(float time, const ct::String& name)
{
    if (name.empty() || !finite(time)) return;
    AnimationEvent event;
    const float limit = mDuration > 0.0f ? mDuration : time;
    event.time = time < 0.0f ? 0.0f : (time > limit ? limit : time);
    event.name = name;
    std::size_t index = 0;
    while (index < mEvents.size() && !(event.time < mEvents[index].time)) ++index;
    mEvents.insert(mEvents.begin() + index, event);
}

void AnimationClip::removeEvent(const ct::String& name)
{
    for (std::size_t i = mEvents.size(); i > 0; --i)
    {
        if (mEvents[i - 1].name == name) mEvents.erase(mEvents.begin() + (i - 1));
    }
}

void AnimationClip::clearEvents()
{
    mEvents.clear();
}

const ct::Vector<AnimationEvent>& AnimationClip::events() const
{
    return mEvents;
}

} // namespace prisma3d
