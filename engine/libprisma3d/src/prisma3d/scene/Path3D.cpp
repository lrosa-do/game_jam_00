#include "prisma3d/scene/Path3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/SceneJson.h"

#include "PathMath.h"
#include "platform.h"

#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

using namespace pathmath;

Path3D::Path3D()
        : Node3D(),
          curveChanged(),
          mCurve(),
          mUpdateCallback()
{
    mCurve.changed = [this]() { onCurveChanged(); };
}

Path3D::~Path3D()
{
    mCurve.changed = ct::Function<void()>();
}

Curve3D& Path3D::getCurve()
{
    return mCurve;
}

const Curve3D& Path3D::getCurve() const
{
    return mCurve;
}

void Path3D::setUpdateCallback(const ct::Function<void()>& callback)
{
    mUpdateCallback = callback;
    setNotifyTransform(static_cast<bool>(mUpdateCallback));
}

void Path3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    if (what == kNotificationTransformChanged)
    {
        if (isInsideTree() && mUpdateCallback) mUpdateCallback();
    }
}

void Path3D::onCurveChanged()
{
    if (isInsideTree())
    {
        if (curveChanged) curveChanged();

        for (std::size_t i = 0; i < getChildCount(); i++)
        {
            PathFollow3D* child = getChild(i)->castTo<PathFollow3D>();
            if (child) child->updateTransform();

            Road3D* road = getChild(i)->castTo<Road3D>();
            if (road) road->onCurveChanged();
        }
    }
}

PathFollow3D::PathFollow3D()
        : Node3D(),
          mPath(nullptr),
          mProgress(0.0f),
          mHOffset(0.0f),
          mVOffset(0.0f),
          mCubic(true),
          mLoop(true),
          mTiltEnabled(true),
          mUseModelFront(false),
          mRotationMode(RotationMode::XYZ)
{
}

PathFollow3D::~PathFollow3D()
{
}

void PathFollow3D::updateTransform()
{
    if (!mPath) return;

    const Curve3D& c = mPath->getCurve();

    const float bl = c.getBakedLength();
    if (bl == 0.0f) return;

    Math::Mat3 basis = Math::Mat3::Identity();
    Math::Vec3 origin(0.0f, 0.0f, 0.0f);

    if (mRotationMode == RotationMode::None)
    {
        origin = c.sampleBaked(mProgress, mCubic);
    }
    else
    {
        Math::Mat4 t = c.sampleBakedWithRotation(mProgress, mCubic, false);
        const Math::Vec3 tangent = -t.col2.xyz();
        t = PathFollow3D::correctPosture(t, mRotationMode);
        basis = t.UpperLeft3x3();
        origin = t.GetTranslation();

        if (mUseModelFront) basis = basis * Math::Mat3::Scale(Math::Vec3(-1.0f, 1.0f, -1.0f));

        if (mTiltEnabled)
        {
            const float tilt = c.sampleBakedTilt(mProgress);

            const Math::Mat3 twist = axisAngle(tangent, tilt);
            basis = twist * basis;
        }
    }

    const Math::Vec3 scale = scaleOf(getBasis());
    origin = origin + basis.col0 * mHOffset + basis.col1 * mVOffset;
    scaleLocal(basis, scale);

    setTransform(Math::Mat4(basis, origin));
}

void PathFollow3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
        {
            Node* parent = getParent();
            if (parent)
            {
                mPath = parent->castTo<Path3D>();
                updateTransform();
            }
            break;
        }
        case kNotificationExitTree:
            mPath = nullptr;
            break;
        default:
            break;
    }
}

void PathFollow3D::setCubicInterpolationEnabled(bool enabled)
{
    mCubic = enabled;
}

bool PathFollow3D::isCubicInterpolationEnabled() const
{
    return mCubic;
}

Math::Mat4 PathFollow3D::correctPosture(const Math::Mat4& transform, RotationMode rotationMode)
{
    Math::Mat3 basis = transform.UpperLeft3x3();
    const Math::Vec3 origin = transform.GetTranslation();

    if (rotationMode == RotationMode::None)
    {
        basis = Math::Mat3::Identity();
    }
    else if (rotationMode == RotationMode::Oriented)
    {
        const Math::Vec3 tangent = -basis.col2;
        basis = lookingAt(tangent);
    }
    else
    {
        Math::Vec3 euler = eulerYXZ(basis);
        if (rotationMode == RotationMode::Y)
        {
            euler[0] = 0.0f;
            euler[2] = 0.0f;
        }
        else if (rotationMode == RotationMode::XY)
        {
            euler[2] = 0.0f;
        }

        basis = basisFromEulerYXZ(euler);
    }

    return Math::Mat4(basis, origin);
}

void PathFollow3D::setProgress(float progress)
{
    if (!isfinite(progress))
    {
        log_error("path follow: progress is non-finite");
        return;
    }
    if (mProgress == progress) return;
    mProgress = progress;

    if (mPath)
    {
        const float pathLength = mPath->getCurve().getBakedLength();

        if (mLoop && pathLength != 0.0f)
        {
            mProgress = fposmod(mProgress, pathLength);
            if (!isZeroApprox(progress) && isZeroApprox(mProgress)) mProgress = pathLength;
        }
        else
        {
            mProgress = clamp(mProgress, 0.0f, pathLength);
        }

        updateTransform();
    }
}

void PathFollow3D::setHOffset(float hOffset)
{
    if (mHOffset == hOffset) return;
    mHOffset = hOffset;
    updateTransform();
}

float PathFollow3D::getHOffset() const
{
    return mHOffset;
}

void PathFollow3D::setVOffset(float vOffset)
{
    if (mVOffset == vOffset) return;
    mVOffset = vOffset;
    updateTransform();
}

float PathFollow3D::getVOffset() const
{
    return mVOffset;
}

float PathFollow3D::getProgress() const
{
    return mProgress;
}

void PathFollow3D::setProgressRatio(float ratio)
{
    if (!mPath)
    {
        log_error("path follow: can only set progress ratio on a follower that is the child of a "
                  "path inside the scene tree");
        return;
    }
    if (mPath->getCurve().getBakedLength() == 0.0f)
    {
        log_error("path follow: can't set progress ratio on a curve with 0 length");
        return;
    }
    setProgress(ratio * mPath->getCurve().getBakedLength());
}

float PathFollow3D::getProgressRatio() const
{
    if (mPath && mPath->getCurve().getBakedLength() != 0.0f)
        return getProgress() / mPath->getCurve().getBakedLength();
    return 0.0f;
}

void PathFollow3D::setRotationMode(RotationMode rotationMode)
{
    if (mRotationMode == rotationMode) return;
    mRotationMode = rotationMode;

    updateTransform();
}

PathFollow3D::RotationMode PathFollow3D::getRotationMode() const
{
    return mRotationMode;
}

void PathFollow3D::setUseModelFront(bool useModelFront)
{
    if (mUseModelFront == useModelFront) return;
    mUseModelFront = useModelFront;
    updateTransform();
}

bool PathFollow3D::isUsingModelFront() const
{
    return mUseModelFront;
}

void PathFollow3D::setLoop(bool loop)
{
    if (mLoop == loop) return;
    mLoop = loop;
    updateTransform();
}

bool PathFollow3D::hasLoop() const
{
    return mLoop;
}

void PathFollow3D::setTiltEnabled(bool enabled)
{
    if (mTiltEnabled == enabled) return;
    mTiltEnabled = enabled;
    updateTransform();
}

bool PathFollow3D::isTiltEnabled() const
{
    return mTiltEnabled;
}

void Path3D::bindProperties()
{
}

void PathFollow3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = PathFollow3D;
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "progress", 0.0, 10000.0, 0.01, true, true),
            &P::setProgress, &P::getProgress);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "h_offset"), &P::setHOffset,
            &P::getHOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "v_offset"), &P::setVOffset,
            &P::getVOffset);
    DB::addProperty<P>(PropertyInfo::enumeration("rotation_mode", "None,Y,XY,XYZ,Oriented"),
            &P::setRotationMode, &P::getRotationMode);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_model_front"), &P::setUseModelFront,
            &P::isUsingModelFront);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "cubic_interp"),
            &P::setCubicInterpolationEnabled, &P::isCubicInterpolationEnabled);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "loop"), &P::setLoop, &P::hasLoop);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "tilt_enabled"), &P::setTiltEnabled,
            &P::isTiltEnabled);
}

namespace
{

constexpr std::int64_t kPath3DDataVersion = 1;

}

bool Path3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    const Curve3D& curve = mCurve;
    const Curve3D defaults;
    if (curve.getPointCount() == 0 && curve.isClosed() == defaults.isClosed() &&
            curve.isUpVectorEnabled() == defaults.isUpVectorEnabled() &&
            curve.getBakeInterval() == defaults.getBakeInterval())
        return false;

    data["version"] = ct::Json(static_cast<long long>(kPath3DDataVersion));
    data["closed"] = ct::Json(curve.isClosed());
    data["bake_interval"] = scenejson::fromFloat(curve.getBakeInterval());
    data["up_vector_enabled"] = ct::Json(curve.isUpVectorEnabled());
    ct::Json points = ct::Json::array();
    for (int i = 0; i < curve.getPointCount(); ++i)
    {
        ct::Json point = ct::Json::object();
        point["position"] = scenejson::fromVec3(curve.getPointPosition(i));
        point["in"] = scenejson::fromVec3(curve.getPointIn(i));
        point["out"] = scenejson::fromVec3(curve.getPointOut(i));
        point["tilt"] = scenejson::fromFloat(curve.getPointTilt(i));
        points.push_back(point);
    }
    data["points"] = points;
    return true;
}

bool Path3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kPath3DDataVersion, result, path)) return false;

    mCurve.clearPoints();
    if (const ct::Json* points = scenejson::arrayField(data, "points", result, path))
    {
        for (std::size_t i = 0; i < points->size(); ++i)
        {
            const ct::Json& entry = points->at(i);
            const ct::String pointPath = scenejson::element(path, "points", i);
            if (!entry.is_object())
            {
                result.addError(pointPath, "expected an object");
                continue;
            }
            Math::Vec3 position(0.0f, 0.0f, 0.0f);
            Math::Vec3 in(0.0f, 0.0f, 0.0f);
            Math::Vec3 out(0.0f, 0.0f, 0.0f);
            float tilt = 0.0f;
            if (!scenejson::readVec3Field(entry, "position", position, result, pointPath))
            {
                if (!entry.contains("position")) result.addError(pointPath, "missing position");
                continue;
            }
            scenejson::readVec3Field(entry, "in", in, result, pointPath);
            scenejson::readVec3Field(entry, "out", out, result, pointPath);
            scenejson::readFloatField(entry, "tilt", tilt, result, pointPath);
            mCurve.addPoint(position, in, out);
            mCurve.setPointTilt(mCurve.getPointCount() - 1, tilt);
        }
    }

    bool flag = false;
    float interval = 0.0f;
    if (scenejson::readBoolField(data, "closed", flag, result, path)) mCurve.setClosed(flag);
    if (scenejson::readBoolField(data, "up_vector_enabled", flag, result, path))
        mCurve.setUpVectorEnabled(flag);
    if (scenejson::readFloatField(data, "bake_interval", interval, result, path))
        mCurve.setBakeInterval(interval);
    return true;
}

} // namespace prisma3d
