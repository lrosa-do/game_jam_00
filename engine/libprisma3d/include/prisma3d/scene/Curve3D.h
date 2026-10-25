#pragma once

#include "mathc.h"

#include <ct/function.hpp>
#include <ct/treemap.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Curve3D
{
public:
    Curve3D();
    Curve3D(const Curve3D& other);
    Curve3D& operator=(const Curve3D& other);
    ~Curve3D();

    int getPointCount() const;
    void setPointCount(int count);
    void addPoint(const Math::Vec3& position, const Math::Vec3& in = Math::Vec3(0.0f, 0.0f, 0.0f),
            const Math::Vec3& out = Math::Vec3(0.0f, 0.0f, 0.0f), int atPosition = -1);
    void setPointPosition(int index, const Math::Vec3& position);
    Math::Vec3 getPointPosition(int index) const;
    void setPointTilt(int index, float tilt);
    float getPointTilt(int index) const;
    void setPointIn(int index, const Math::Vec3& in);
    Math::Vec3 getPointIn(int index) const;
    void setPointOut(int index, const Math::Vec3& out);
    Math::Vec3 getPointOut(int index) const;
    void removePoint(int index);
    void clearPoints();

    Math::Vec3 sample(int index, float offset) const;
    Math::Vec3 samplef(float findex) const;

    void setClosed(bool closed);
    bool isClosed() const;
    void setBakeInterval(float tolerance);
    float getBakeInterval() const;
    void setUpVectorEnabled(bool enable);
    bool isUpVectorEnabled() const;

    float getBakedLength() const;
    Math::Vec3 sampleBaked(float offset, bool cubic = false) const;
    Math::Mat4 sampleBakedWithRotation(float offset, bool cubic = false,
            bool applyTilt = false) const;
    float sampleBakedTilt(float offset) const;
    Math::Vec3 sampleBakedUpVector(float offset, bool applyTilt = false) const;
    const ct::Vector<Math::Vec3>& getBakedPoints() const;
    const ct::Vector<float>& getBakedTilts() const;
    const ct::Vector<Math::Vec3>& getBakedUpVectors() const;
    const ct::Vector<float>& getBakedDistCache() const;
    Math::Vec3 getClosestPoint(const Math::Vec3& toPoint) const;
    float getClosestOffset(const Math::Vec3& toPoint) const;
    ct::Vector<Math::Vec3> getPoints() const;

    ct::Vector<Math::Vec3> tessellate(int maxStages = 5, float tolerance = 4.0f) const;
    ct::Vector<Math::Vec3> tessellateEvenLength(int maxStages = 5, float length = 0.2f) const;

    ct::Function<void()> changed;

private:
    struct Point
    {
        Math::Vec3 in = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec3 out = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
        float tilt = 0.0f;
    };

    struct Interval
    {
        int idx;
        float frac;
    };

    using Segment = ct::TreeMap<float, Math::Vec3>;

    void markDirty();
    void addPointInternal(const Math::Vec3& position, const Math::Vec3& in, const Math::Vec3& out,
            int atPosition);
    void removePointInternal(int index);

    static Math::Vec3 calculateTangent(const Math::Vec3& begin, const Math::Vec3& control1,
            const Math::Vec3& control2, const Math::Vec3& end, float t);
    void bake() const;

    Interval findInterval(float offset) const;
    Math::Vec3 sampleBakedAt(Interval interval, bool cubic) const;
    float sampleBakedTiltAt(Interval interval) const;
    Math::Mat3 samplePosture(Interval interval, bool applyTilt) const;
    Math::Mat3 composePosture(int index) const;

    static void bakeSegment3D(Segment& bake, float begin, float end, const Math::Vec3& a,
            const Math::Vec3& out, const Math::Vec3& b, const Math::Vec3& in, int depth,
            int maxDepth, float tolerance);
    static void bakeSegment3DEvenLength(Segment& bake, float begin, float end, const Math::Vec3& a,
            const Math::Vec3& out, const Math::Vec3& b, const Math::Vec3& in, int depth,
            int maxDepth, float length);

    ct::Vector<Segment> tessellateEvenLengthSegments(int maxStages, float length) const;

    ct::Vector<Point> mPoints;
    bool mClosed;

    mutable bool mBakedCacheDirty;
    mutable ct::Vector<Math::Vec3> mBakedPointCache;
    mutable ct::Vector<float> mBakedTiltCache;
    mutable ct::Vector<Math::Vec3> mBakedUpVectorCache;
    mutable ct::Vector<Math::Vec3> mBakedForwardVectorCache;
    mutable ct::Vector<float> mBakedDistCache;
    mutable float mBakedMaxOfs;

    float mBakeInterval;
    bool mUpVectorEnabled;
};

} // namespace prisma3d
