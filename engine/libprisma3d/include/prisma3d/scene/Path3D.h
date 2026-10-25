#pragma once

#include "prisma3d/scene/Curve3D.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/function.hpp>

namespace prisma3d
{

class Path3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Path3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    Path3D();
    ~Path3D() override;

    Curve3D& getCurve();
    const Curve3D& getCurve() const;

    void setUpdateCallback(const ct::Function<void()>& callback);

    ct::Function<void()> curveChanged;

protected:
    void notificationHandler(int what) override;

private:
    void onCurveChanged();

    Curve3D mCurve;
    ct::Function<void()> mUpdateCallback;
};

class PathFollow3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(PathFollow3D, Node3D);
    static void bindProperties();

    enum class RotationMode
    {
        None,
        Y,
        XY,
        XYZ,
        Oriented
    };

    PathFollow3D();
    ~PathFollow3D() override;

    void setProgress(float progress);
    float getProgress() const;

    void setHOffset(float hOffset);
    float getHOffset() const;

    void setVOffset(float vOffset);
    float getVOffset() const;

    void setProgressRatio(float ratio);
    float getProgressRatio() const;

    void setLoop(bool loop);
    bool hasLoop() const;

    void setTiltEnabled(bool enabled);
    bool isTiltEnabled() const;

    void setRotationMode(RotationMode rotationMode);
    RotationMode getRotationMode() const;

    void setUseModelFront(bool useModelFront);
    bool isUsingModelFront() const;

    void setCubicInterpolationEnabled(bool enabled);
    bool isCubicInterpolationEnabled() const;

    void updateTransform();

    static Math::Mat4 correctPosture(const Math::Mat4& transform, RotationMode rotationMode);

protected:
    void notificationHandler(int what) override;

private:
    Path3D* mPath;
    float mProgress;
    float mHOffset;
    float mVOffset;
    bool mCubic;
    bool mLoop;
    bool mTiltEnabled;
    bool mUseModelFront;
    RotationMode mRotationMode;
};

} // namespace prisma3d
