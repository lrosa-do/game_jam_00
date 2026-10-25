#include "prisma3d/scene/Curve3D.h"

#include "PathMath.h"
#include "platform.h"

#include <float.h>
#include <math.h>

namespace prisma3d
{

using namespace pathmath;

Curve3D::Curve3D()
        : changed(),
          mPoints(),
          mClosed(false),
          mBakedCacheDirty(false),
          mBakedPointCache(),
          mBakedTiltCache(),
          mBakedUpVectorCache(),
          mBakedForwardVectorCache(),
          mBakedDistCache(),
          mBakedMaxOfs(0.0f),
          mBakeInterval(0.2f),
          mUpVectorEnabled(true)
{
}

Curve3D::Curve3D(const Curve3D& other)
        : changed(),
          mPoints(other.mPoints),
          mClosed(other.mClosed),
          mBakedCacheDirty(true),
          mBakedPointCache(),
          mBakedTiltCache(),
          mBakedUpVectorCache(),
          mBakedForwardVectorCache(),
          mBakedDistCache(),
          mBakedMaxOfs(0.0f),
          mBakeInterval(other.mBakeInterval),
          mUpVectorEnabled(other.mUpVectorEnabled)
{
}

Curve3D& Curve3D::operator=(const Curve3D& other)
{
    if (this == &other) return *this;
    mPoints = other.mPoints;
    mClosed = other.mClosed;
    mBakeInterval = other.mBakeInterval;
    mUpVectorEnabled = other.mUpVectorEnabled;
    markDirty();
    return *this;
}

Curve3D::~Curve3D()
{
}

int Curve3D::getPointCount() const
{
    return static_cast<int>(mPoints.size());
}

void Curve3D::setPointCount(int count)
{
    if (count < 0)
    {
        log_error("curve3d: point count %d is negative", count);
        return;
    }
    const int oldSize = static_cast<int>(mPoints.size());
    if (oldSize == count) return;

    if (oldSize > count)
    {
        mPoints.resize(static_cast<std::size_t>(count));
        markDirty();
    }
    else
    {
        for (int i = count - oldSize; i > 0; i--)
            addPointInternal(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
                    Math::Vec3(0.0f, 0.0f, 0.0f), -1);
    }
}

void Curve3D::addPointInternal(const Math::Vec3& position, const Math::Vec3& in,
        const Math::Vec3& out, int atPosition)
{
    Point n;
    n.position = position;
    n.in = in;
    n.out = out;
    if (static_cast<std::uint32_t>(atPosition) < mPoints.size())
        mPoints.insert(mPoints.begin() + atPosition, n);
    else
        mPoints.push_back(n);

    markDirty();
}

void Curve3D::addPoint(const Math::Vec3& position, const Math::Vec3& in, const Math::Vec3& out,
        int atPosition)
{
    addPointInternal(position, in, out, atPosition);
}

void Curve3D::setPointPosition(int index, const Math::Vec3& position)
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return;
    }

    mPoints[index].position = position;
    markDirty();
}

Math::Vec3 Curve3D::getPointPosition(int index) const
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    return mPoints[index].position;
}

void Curve3D::setPointTilt(int index, float tilt)
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return;
    }

    mPoints[index].tilt = tilt;
    markDirty();
}

float Curve3D::getPointTilt(int index) const
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return 0.0f;
    }
    return mPoints[index].tilt;
}

void Curve3D::setPointIn(int index, const Math::Vec3& in)
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return;
    }

    mPoints[index].in = in;
    markDirty();
}

Math::Vec3 Curve3D::getPointIn(int index) const
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    return mPoints[index].in;
}

void Curve3D::setPointOut(int index, const Math::Vec3& out)
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return;
    }

    mPoints[index].out = out;
    markDirty();
}

Math::Vec3 Curve3D::getPointOut(int index) const
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    return mPoints[index].out;
}

void Curve3D::removePointInternal(int index)
{
    if (static_cast<std::uint32_t>(index) >= mPoints.size())
    {
        log_error("curve3d: point index %d out of range", index);
        return;
    }
    mPoints.erase(mPoints.begin() + index);
    markDirty();
}

void Curve3D::removePoint(int index)
{
    removePointInternal(index);
    if (mClosed && mPoints.size() < 2) setClosed(false);
}

void Curve3D::clearPoints()
{
    if (!mPoints.empty())
    {
        mPoints.clear();
        markDirty();
    }
}

Math::Vec3 Curve3D::sample(int index, float offset) const
{
    const int pc = static_cast<int>(mPoints.size());
    if (pc == 0)
    {
        log_error("curve3d: sample on a curve without points");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    if (index >= pc - 1)
    {
        if (!mClosed) return mPoints[pc - 1].position;
        index = pc - 1;
    }
    else if (index < 0)
    {
        return mPoints[0].position;
    }

    const Math::Vec3 p0 = mPoints[index].position;
    const Math::Vec3 p1 = p0 + mPoints[index].out;
    Math::Vec3 p3, p2;
    if (!mClosed || index < pc - 1)
    {
        p3 = mPoints[index + 1].position;
        p2 = p3 + mPoints[index + 1].in;
    }
    else
    {
        p3 = mPoints[0].position;
        p2 = p3 + mPoints[0].in;
    }

    return bezierInterpolate(p0, p1, p2, p3, offset);
}

Math::Vec3 Curve3D::samplef(float findex) const
{
    if (findex < 0.0f) findex = 0.0f;
    else if (findex >= static_cast<float>(mPoints.size()))
        findex = static_cast<float>(mPoints.size());

    return sample(static_cast<int>(findex), fmodf(findex, 1.0f));
}

void Curve3D::markDirty()
{
    mBakedCacheDirty = true;
    if (changed) changed();
}

void Curve3D::bakeSegment3D(Segment& bake, float begin, float end, const Math::Vec3& a,
        const Math::Vec3& out, const Math::Vec3& b, const Math::Vec3& in, int depth, int maxDepth,
        float tolerance)
{
    const float mp = begin + (end - begin) * 0.5f;
    const Math::Vec3 beg = bezierInterpolate(a, a + out, b + in, b, begin);
    const Math::Vec3 mid = bezierInterpolate(a, a + out, b + in, b, mp);
    const Math::Vec3 fin = bezierInterpolate(a, a + out, b + in, b, end);

    const Math::Vec3 na = normalized(mid - beg);
    const Math::Vec3 nb = normalized(fin - mid);
    const float dp = na.Dot(nb);

    if (dp < cosf(tolerance * (kPi / 180.0f))) bake[mp] = mid;
    if (depth < maxDepth)
    {
        bakeSegment3D(bake, begin, mp, a, out, b, in, depth + 1, maxDepth, tolerance);
        bakeSegment3D(bake, mp, end, a, out, b, in, depth + 1, maxDepth, tolerance);
    }
}

void Curve3D::bakeSegment3DEvenLength(Segment& bake, float begin, float end, const Math::Vec3& a,
        const Math::Vec3& out, const Math::Vec3& b, const Math::Vec3& in, int depth, int maxDepth,
        float length)
{
    const Math::Vec3 beg = bezierInterpolate(a, a + out, b + in, b, begin);
    const Math::Vec3 fin = bezierInterpolate(a, a + out, b + in, b, end);

    const float segmentLength = (fin - beg).Length();

    if (segmentLength > length && depth < maxDepth)
    {
        const float mp = (begin + end) * 0.5f;
        const Math::Vec3 mid = bezierInterpolate(a, a + out, b + in, b, mp);
        bake[mp] = mid;

        bakeSegment3DEvenLength(bake, begin, mp, a, out, b, in, depth + 1, maxDepth, length);
        bakeSegment3DEvenLength(bake, mp, end, a, out, b, in, depth + 1, maxDepth, length);
    }
}

Math::Vec3 Curve3D::calculateTangent(const Math::Vec3& begin, const Math::Vec3& control1,
        const Math::Vec3& control2, const Math::Vec3& end, float t)
{
    if (isZeroApprox(t - 0.0f))
    {
        if (isEqualApprox(control1, begin))
        {
            if (isEqualApprox(control1, control2)) return normalized(end - begin);
            return normalized(control2 - begin);
        }
    }
    else if (isZeroApprox(t - 1.0f))
    {
        if (isEqualApprox(control2, end))
        {
            if (isEqualApprox(control2, control1)) return normalized(end - begin);
            return normalized(end - control1);
        }
    }

    if (isEqualApprox(control1, end) && isEqualApprox(control2, begin))
        return normalized(end - begin);

    return normalized(bezierDerivative(begin, control1, control2, end, t));
}

void Curve3D::bake() const
{
    if (!mBakedCacheDirty) return;

    mBakedMaxOfs = 0.0f;
    mBakedCacheDirty = false;

    if (mPoints.empty())
    {
        mBakedPointCache.clear();
        mBakedTiltCache.clear();
        mBakedDistCache.clear();

        mBakedForwardVectorCache.clear();
        mBakedUpVectorCache.clear();
        return;
    }

    if (mPoints.size() == 1)
    {
        mBakedPointCache.resize(1);
        mBakedPointCache[0] = mPoints[0].position;
        mBakedTiltCache.resize(1);
        mBakedTiltCache[0] = mPoints[0].tilt;
        mBakedDistCache.resize(1);
        mBakedDistCache[0] = 0.0f;
        mBakedForwardVectorCache.resize(1);
        mBakedForwardVectorCache[0] = Math::Vec3(0.0f, 0.0f, 1.0f);

        if (mUpVectorEnabled)
        {
            mBakedUpVectorCache.resize(1);
            mBakedUpVectorCache[0] = Math::Vec3(0.0f, 1.0f, 0.0f);
        }
        else
        {
            mBakedUpVectorCache.clear();
        }

        return;
    }

    {
        ct::Vector<Segment> midpoints = tessellateEvenLengthSegments(10, mBakeInterval);

        const int numIntervals =
                mClosed ? static_cast<int>(mPoints.size()) : static_cast<int>(mPoints.size()) - 1;

        int pc = 1;
        for (int i = 0; i < numIntervals; i++)
        {
            pc++;
            pc += static_cast<int>(midpoints[i].size());
        }

        mBakedPointCache.resize(pc);
        mBakedTiltCache.resize(pc);
        mBakedDistCache.resize(pc);
        mBakedForwardVectorCache.resize(pc);

        Math::Vec3* bpw = mBakedPointCache.data();
        float* btw = mBakedTiltCache.data();
        Math::Vec3* bfw = mBakedForwardVectorCache.data();

        bpw[0] = mPoints[0].position;
        bfw[0] = calculateTangent(mPoints[0].position, mPoints[0].position + mPoints[0].out,
                mPoints[1].position + mPoints[1].in, mPoints[1].position, 0.0f);
        btw[0] = mPoints[0].tilt;
        int pidx = 0;

        for (int i = 0; i < numIntervals; i++)
        {
            for (Segment::iterator it = midpoints[i].begin(); it != midpoints[i].end(); ++it)
            {
                pidx++;
                bpw[pidx] = it->value;
                if (!mClosed || i < numIntervals - 1)
                {
                    bfw[pidx] = calculateTangent(mPoints[i].position,
                            mPoints[i].position + mPoints[i].out,
                            mPoints[i + 1].position + mPoints[i + 1].in, mPoints[i + 1].position,
                            it->key);
                    btw[pidx] = lerp(mPoints[i].tilt, mPoints[i + 1].tilt, it->key);
                }
                else
                {
                    bfw[pidx] = calculateTangent(mPoints[i].position,
                            mPoints[i].position + mPoints[i].out,
                            mPoints[0].position + mPoints[0].in, mPoints[0].position, it->key);
                    btw[pidx] = lerp(mPoints[i].tilt, mPoints[0].tilt, it->key);
                }
            }

            pidx++;
            if (!mClosed || i < numIntervals - 1)
            {
                bpw[pidx] = mPoints[i + 1].position;
                bfw[pidx] = calculateTangent(mPoints[i].position,
                        mPoints[i].position + mPoints[i].out,
                        mPoints[i + 1].position + mPoints[i + 1].in, mPoints[i + 1].position, 1.0f);
                btw[pidx] = mPoints[i + 1].tilt;
            }
            else
            {
                bpw[pidx] = mPoints[0].position;
                bfw[pidx] =
                        calculateTangent(mPoints[i].position, mPoints[i].position + mPoints[i].out,
                                mPoints[0].position + mPoints[0].in, mPoints[0].position, 1.0f);
                btw[pidx] = mPoints[0].tilt;
            }
        }

        float* bdw = mBakedDistCache.data();
        bdw[0] = 0.0f;
        for (int i = 0; i < pc - 1; i++) bdw[i + 1] = bdw[i] + (bpw[i + 1] - bpw[i]).Length();
        mBakedMaxOfs = bdw[pc - 1];
    }

    if (!mUpVectorEnabled)
    {
        mBakedUpVectorCache.resize(0);
        return;
    }

    {
        const int pointCount = static_cast<int>(mBakedPointCache.size());

        mBakedUpVectorCache.resize(pointCount);
        Math::Vec3* upWrite = mBakedUpVectorCache.data();

        const Math::Vec3* forwardPtr = mBakedForwardVectorCache.data();
        const Math::Vec3* pointsPtr = mBakedPointCache.data();

        Math::Mat3 frame = Math::Mat3::Identity();
        Math::Mat3 framePrev = Math::Mat3::Identity();

        {
            const Math::Vec3 forward = forwardPtr[0];

            if (fabsf(forward.Dot(Math::Vec3(0.0f, 1.0f, 0.0f))) > 1.0 - kUnitEpsilon)
                framePrev = lookingAt(forward, Math::Vec3(1.0f, 0.0f, 0.0f));
            else
                framePrev = lookingAt(forward, Math::Vec3(0.0f, 1.0f, 0.0f));

            upWrite[0] = framePrev.col1;
        }

        for (int idx = 1; idx < pointCount; idx++)
        {
            const Math::Vec3 forward = forwardPtr[idx];

            Math::Mat3 rotate = Math::Mat3::Identity();
            rotateToAlign(rotate, -framePrev.col2, forward);
            frame = rotate * framePrev;
            frame = orthonormalized(frame);

            upWrite[idx] = frame.col1;
            framePrev = frame;
        }

        bool isLoop = true;
        {
            if (!isEqualApprox(pointsPtr[0], pointsPtr[pointCount - 1])) isLoop = false;

            const float dot = forwardPtr[0].Dot(forwardPtr[pointCount - 1]);
            if (dot < 1.0 - kUnitEpsilon) isLoop = false;
        }

        if (isLoop)
        {
            const Math::Vec3 upStart = upWrite[0];
            const Math::Vec3 upEnd = upWrite[pointCount - 1];

            const float sign = signOf(upEnd.Cross(upStart).Dot(forwardPtr[0]));
            const float fullAngle = shortestArcAngle(upEnd, upStart);

            if (fabsf(fullAngle) < kCmpEpsilon)
            {
                return;
            }
            else
            {
                const float* dists = mBakedDistCache.data();
                for (int idx = 1; idx < pointCount; idx++)
                {
                    const float frac = dists[idx] / mBakedMaxOfs;
                    const float angle = lerp(0.0f, fullAngle, frac);
                    const Math::Mat3 twist = axisAngle(forwardPtr[idx] * sign, angle);

                    upWrite[idx] = twist * upWrite[idx];
                }
            }
        }
    }
}

float Curve3D::getBakedLength() const
{
    if (mBakedCacheDirty) bake();

    return mBakedMaxOfs;
}

Curve3D::Interval Curve3D::findInterval(float offset) const
{
    Interval interval = { -1, 0.0f };
    if (mBakedCacheDirty)
    {
        log_error("curve3d: baked cache is dirty");
        return interval;
    }

    const int pc = static_cast<int>(mBakedPointCache.size());
    if (pc < 2)
    {
        log_error("curve3d: less than two points in cache");
        return interval;
    }

    int start = 0;
    int end = pc;
    int idx = (end + start) / 2;
    while (start < idx)
    {
        const float distance = mBakedDistCache[idx];
        if (offset <= distance) end = idx;
        else
            start = idx;
        idx = (end + start) / 2;
    }

    const float offsetBegin = mBakedDistCache[idx];
    const float offsetEnd = mBakedDistCache[idx + 1];

    const float idxInterval = offsetEnd - offsetBegin;
    if (offset < offsetBegin || offset > offsetEnd)
    {
        log_error("curve3d: offset out of range");
        return interval;
    }

    interval.idx = idx;
    if (idxInterval < FLT_EPSILON)
    {
        interval.frac = 0.5f;
        log_error("curve3d: zero length interval");
        return interval;
    }

    interval.frac = (offset - offsetBegin) / idxInterval;
    return interval;
}

Math::Vec3 Curve3D::sampleBakedAt(Interval interval, bool cubic) const
{
    if (interval.idx < 0 || interval.idx >= static_cast<int>(mBakedPointCache.size()))
    {
        log_error("curve3d: invalid interval");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    const int idx = interval.idx;
    const float frac = interval.frac;

    const Math::Vec3* r = mBakedPointCache.data();
    const int pc = static_cast<int>(mBakedPointCache.size());

    if (cubic)
    {
        const Math::Vec3 pre = idx > 0 ? r[idx - 1] : r[idx];
        const Math::Vec3 post = (idx < (pc - 2)) ? r[idx + 2] : r[idx + 1];
        return cubicInterpolate(r[idx], r[idx + 1], pre, post, frac);
    }
    return lerp(r[idx], r[idx + 1], frac);
}

float Curve3D::sampleBakedTiltAt(Interval interval) const
{
    if (interval.idx < 0 || interval.idx >= static_cast<int>(mBakedTiltCache.size()))
    {
        log_error("curve3d: invalid interval");
        return 0.0f;
    }

    const int idx = interval.idx;
    const float frac = interval.frac;

    const float* r = mBakedTiltCache.data();

    return lerp(r[idx], r[idx + 1], frac);
}

Math::Mat3 Curve3D::composePosture(int index) const
{
    const Math::Vec3 forward = mBakedForwardVectorCache[index];

    Math::Vec3 up;
    if (mUpVectorEnabled) up = mBakedUpVectorCache[index];
    else
        up = Math::Vec3(0.0f, 1.0f, 0.0f);

    return lookingAt(forward, up);
}

Math::Mat3 Curve3D::samplePosture(Interval interval, bool applyTilt) const
{
    if (interval.idx < 0 || interval.idx >= static_cast<int>(mBakedPointCache.size()))
    {
        log_error("curve3d: invalid interval");
        return Math::Mat3::Identity();
    }
    if (mUpVectorEnabled)
    {
        if (interval.idx >= static_cast<int>(mBakedUpVectorCache.size()))
        {
            log_error("curve3d: invalid interval");
            return Math::Mat3::Identity();
        }
    }

    const int idx = interval.idx;
    const float frac = interval.frac;

    const Math::Mat3 frameBegin = composePosture(idx);
    const Math::Mat3 frameEnd = composePosture(idx + 1);
    const Math::Mat3 frame = orthonormalized(slerp(frameBegin, frameEnd, frac));

    if (!applyTilt) return frame;

    const float tilt = sampleBakedTiltAt(interval);
    const Math::Vec3 tangent = -frame.col2;

    const Math::Mat3 twist = axisAngle(tangent, tilt);
    return twist * frame;
}

Math::Vec3 Curve3D::sampleBaked(float offset, bool cubic) const
{
    if (!isfinite(offset))
    {
        log_error("curve3d: offset is non-finite");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    if (mBakedCacheDirty) bake();

    const int pc = static_cast<int>(mBakedPointCache.size());
    if (pc == 0)
    {
        log_error("curve3d: no points in curve");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    if (pc == 1) return mBakedPointCache[0];

    offset = clamp(offset, 0.0f, getBakedLength());

    const Interval interval = findInterval(offset);
    return sampleBakedAt(interval, cubic);
}

Math::Mat4 Curve3D::sampleBakedWithRotation(float offset, bool cubic, bool applyTilt) const
{
    if (!isfinite(offset))
    {
        log_error("curve3d: offset is non-finite");
        return Math::Mat4::Identity();
    }

    if (mBakedCacheDirty) bake();

    const int pointCount = static_cast<int>(mBakedPointCache.size());
    if (pointCount == 0)
    {
        log_error("curve3d: no points in curve");
        return Math::Mat4::Identity();
    }

    if (pointCount == 1)
    {
        log_error("curve3d: only one point in curve");
        return Math::Mat4::Translation(mBakedPointCache[0]);
    }

    offset = clamp(offset, 0.0f, getBakedLength());

    const Interval interval = findInterval(offset);

    const Math::Vec3 pos = sampleBakedAt(interval, cubic);

    const Math::Mat3 frame = samplePosture(interval, applyTilt);

    return Math::Mat4(frame, pos);
}

float Curve3D::sampleBakedTilt(float offset) const
{
    if (!isfinite(offset))
    {
        log_error("curve3d: offset is non-finite");
        return 0.0f;
    }

    if (mBakedCacheDirty) bake();

    const int pc = static_cast<int>(mBakedTiltCache.size());
    if (pc == 0)
    {
        log_error("curve3d: no tilts in curve");
        return 0.0f;
    }

    if (pc == 1) return mBakedTiltCache[0];

    offset = clamp(offset, 0.0f, getBakedLength());

    const Interval interval = findInterval(offset);
    return sampleBakedTiltAt(interval);
}

Math::Vec3 Curve3D::sampleBakedUpVector(float offset, bool applyTilt) const
{
    if (!isfinite(offset))
    {
        log_error("curve3d: offset is non-finite");
        return Math::Vec3(0.0f, 1.0f, 0.0f);
    }

    if (mBakedCacheDirty) bake();

    if (!mUpVectorEnabled)
    {
        log_error("curve3d: no up vectors in curve");
        return Math::Vec3(0.0f, 1.0f, 0.0f);
    }

    const int count = static_cast<int>(mBakedUpVectorCache.size());
    if (count == 1) return mBakedUpVectorCache[0];

    offset = clamp(offset, 0.0f, getBakedLength());

    const Interval interval = findInterval(offset);
    return samplePosture(interval, applyTilt).col1;
}

const ct::Vector<Math::Vec3>& Curve3D::getBakedPoints() const
{
    if (mBakedCacheDirty) bake();

    return mBakedPointCache;
}

const ct::Vector<float>& Curve3D::getBakedTilts() const
{
    if (mBakedCacheDirty) bake();

    return mBakedTiltCache;
}

const ct::Vector<Math::Vec3>& Curve3D::getBakedUpVectors() const
{
    if (mBakedCacheDirty) bake();

    return mBakedUpVectorCache;
}

const ct::Vector<float>& Curve3D::getBakedDistCache() const
{
    if (mBakedCacheDirty) bake();

    return mBakedDistCache;
}

Math::Vec3 Curve3D::getClosestPoint(const Math::Vec3& toPoint) const
{
    if (mBakedCacheDirty) bake();

    const int pc = static_cast<int>(mBakedPointCache.size());
    if (pc == 0)
    {
        log_error("curve3d: no points in curve");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    if (pc == 1) return mBakedPointCache[0];

    const Math::Vec3* r = mBakedPointCache.data();

    Math::Vec3 nearest(0.0f, 0.0f, 0.0f);
    float nearestDist = -1.0f;

    for (int i = 0; i < pc - 1; i++)
    {
        const float interval = mBakedDistCache[i + 1] - mBakedDistCache[i];
        const Math::Vec3 origin = r[i];
        const Math::Vec3 direction = (r[i + 1] - origin) / interval;

        const float d = clamp((toPoint - origin).Dot(direction), 0.0f, interval);
        const Math::Vec3 proj = origin + direction * d;

        const float dist = (toPoint - proj).LengthSquared();

        if (nearestDist < 0.0f || dist < nearestDist)
        {
            nearest = proj;
            nearestDist = dist;
        }
    }

    return nearest;
}

ct::Vector<Math::Vec3> Curve3D::getPoints() const
{
    ct::Vector<Math::Vec3> data;
    data.resize(mPoints.size() * 3);
    for (std::size_t i = 0; i < mPoints.size(); i++)
    {
        data[i * 3 + 0] = mPoints[i].in;
        data[i * 3 + 1] = mPoints[i].out;
        data[i * 3 + 2] = mPoints[i].position;
    }
    return data;
}

float Curve3D::getClosestOffset(const Math::Vec3& toPoint) const
{
    if (mBakedCacheDirty) bake();

    const int pc = static_cast<int>(mBakedPointCache.size());
    if (pc == 0)
    {
        log_error("curve3d: no points in curve");
        return 0.0f;
    }

    if (pc == 1) return 0.0f;

    const Math::Vec3* r = mBakedPointCache.data();

    float nearest = 0.0f;
    float nearestDist = -1.0f;
    float offset;

    for (int i = 0; i < pc - 1; i++)
    {
        offset = mBakedDistCache[i];

        const float interval = mBakedDistCache[i + 1] - mBakedDistCache[i];
        const Math::Vec3 origin = r[i];
        const Math::Vec3 direction = (r[i + 1] - origin) / interval;

        const float d = clamp((toPoint - origin).Dot(direction), 0.0f, interval);
        const Math::Vec3 proj = origin + direction * d;

        const float dist = (toPoint - proj).LengthSquared();

        if (nearestDist < 0.0f || dist < nearestDist)
        {
            nearest = offset + d;
            nearestDist = dist;
        }
    }

    return nearest;
}

void Curve3D::setClosed(bool closed)
{
    if (mClosed == closed) return;

    mClosed = closed;
    markDirty();
}

bool Curve3D::isClosed() const
{
    return mClosed;
}

void Curve3D::setBakeInterval(float tolerance)
{
    mBakeInterval = tolerance;
    markDirty();
}

float Curve3D::getBakeInterval() const
{
    return mBakeInterval;
}

void Curve3D::setUpVectorEnabled(bool enable)
{
    mUpVectorEnabled = enable;
    markDirty();
}

bool Curve3D::isUpVectorEnabled() const
{
    return mUpVectorEnabled;
}

ct::Vector<Math::Vec3> Curve3D::tessellate(int maxStages, float tolerance) const
{
    ct::Vector<Math::Vec3> tess;

    if (mPoints.empty()) return tess;
    ct::Vector<Segment> midpoints;

    const int numIntervals =
            mClosed ? static_cast<int>(mPoints.size()) : static_cast<int>(mPoints.size()) - 1;
    midpoints.resize(numIntervals);

    int pc = 1;
    for (int i = 0; i < numIntervals; i++)
    {
        if (!mClosed || i < numIntervals - 1)
            bakeSegment3D(midpoints[i], 0.0f, 1.0f, mPoints[i].position, mPoints[i].out,
                    mPoints[i + 1].position, mPoints[i + 1].in, 0, maxStages, tolerance);
        else
            bakeSegment3D(midpoints[i], 0.0f, 1.0f, mPoints[i].position, mPoints[i].out,
                    mPoints[0].position, mPoints[0].in, 0, maxStages, tolerance);
        pc++;
        pc += static_cast<int>(midpoints[i].size());
    }

    tess.resize(pc);
    Math::Vec3* bpw = tess.data();
    bpw[0] = mPoints[0].position;
    int pidx = 0;

    for (int i = 0; i < numIntervals; i++)
    {
        for (Segment::iterator it = midpoints[i].begin(); it != midpoints[i].end(); ++it)
        {
            pidx++;
            bpw[pidx] = it->value;
        }

        pidx++;
        if (!mClosed || i < numIntervals - 1) bpw[pidx] = mPoints[i + 1].position;
        else
            bpw[pidx] = mPoints[0].position;
    }

    return tess;
}

ct::Vector<Curve3D::Segment> Curve3D::tessellateEvenLengthSegments(int maxStages,
        float length) const
{
    ct::Vector<Segment> midpoints;
    if (mPoints.size() < 2)
    {
        log_error("curve3d: curve must have at least 2 control points");
        return midpoints;
    }

    const int numIntervals =
            mClosed ? static_cast<int>(mPoints.size()) : static_cast<int>(mPoints.size()) - 1;
    midpoints.resize(numIntervals);

    for (int i = 0; i < numIntervals; i++)
    {
        if (!mClosed || i < numIntervals - 1)
            bakeSegment3DEvenLength(midpoints[i], 0.0f, 1.0f, mPoints[i].position, mPoints[i].out,
                    mPoints[i + 1].position, mPoints[i + 1].in, 0, maxStages, length);
        else
            bakeSegment3DEvenLength(midpoints[i], 0.0f, 1.0f, mPoints[i].position, mPoints[i].out,
                    mPoints[0].position, mPoints[0].in, 0, maxStages, length);
    }
    return midpoints;
}

ct::Vector<Math::Vec3> Curve3D::tessellateEvenLength(int maxStages, float length) const
{
    ct::Vector<Math::Vec3> tess;

    ct::Vector<Segment> midpoints = tessellateEvenLengthSegments(maxStages, length);
    if (midpoints.empty()) return tess;

    const int numIntervals =
            mClosed ? static_cast<int>(mPoints.size()) : static_cast<int>(mPoints.size()) - 1;
    int pc = 1;
    for (int i = 0; i < numIntervals; i++)
    {
        pc++;
        pc += static_cast<int>(midpoints[i].size());
    }

    tess.resize(pc);
    Math::Vec3* bpw = tess.data();
    bpw[0] = mPoints[0].position;
    int pidx = 0;

    for (int i = 0; i < numIntervals; i++)
    {
        for (Segment::iterator it = midpoints[i].begin(); it != midpoints[i].end(); ++it)
        {
            pidx++;
            bpw[pidx] = it->value;
        }

        pidx++;
        if (!mClosed || i < numIntervals - 1) bpw[pidx] = mPoints[i + 1].position;
        else
            bpw[pidx] = mPoints[0].position;
    }

    return tess;
}

} // namespace prisma3d
