#pragma once

#include <ct/function.hpp>
#include <ct/vector.hpp>

#include "mathc.h"

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class Node;
class Node3D;
class SceneTree;
class Tween;

enum class TweenProcessMode : std::uint8_t
{
    Physics,
    Idle
};

enum class TweenPauseMode : std::uint8_t
{
    Bound,
    Stop,
    Process
};

enum class TransitionType : std::uint8_t
{
    Linear,
    Sine,
    Quint,
    Quart,
    Quad,
    Expo,
    Elastic,
    Cubic,
    Circ,
    Bounce,
    Back,
    Spring,
    Max
};

enum class EaseType : std::uint8_t
{
    In,
    Out,
    InOut,
    OutIn,
    Max
};

class TweenValue
{
public:
    enum class Type : std::uint8_t
    {
        Float,
        Vec3,
        Quaternion,
        Color
    };

    TweenValue();
    TweenValue(double value);
    TweenValue(float value);
    TweenValue(int value);
    TweenValue(const Math::Vec3& value);
    TweenValue(const Math::Quaternion& value);
    TweenValue(const Math::Vec4& color);

    Type type() const;
    double asFloat() const;
    Math::Vec3 asVec3() const;
    Math::Quaternion asQuaternion() const;
    Math::Vec4 asColor() const;

private:
    Type mType;
    double mFloat;
    Math::Vec4 mVector;
};

class TweenKey
{
private:
    friend class Tween;
    friend class SceneTree;
    TweenKey()
    {
    }
};

class Tweener
{
public:
    virtual ~Tweener();

    ct::Function<void()> finished;

protected:
    Tweener();

    virtual void setTween(Tween* tween);
    virtual void start();
    virtual bool step(double& delta) = 0;
    virtual void dropNode(Node* node);

    Tween* getTween() const;
    void finish();

    double mElapsedTime;
    bool mFinished;

private:
    friend class Tween;

    Tween* mTween;
};

class PropertyTweener : public Tweener
{
public:
    PropertyTweener(TweenKey key, ct::Function<TweenValue()> getter,
            ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration);

    PropertyTweener* from(const TweenValue& value);
    PropertyTweener* fromCurrent();
    PropertyTweener* asRelative();
    PropertyTweener* setTrans(TransitionType trans);
    PropertyTweener* setEase(EaseType ease);
    PropertyTweener* setCustomInterpolator(ct::Function<float(float)> interpolator);
    PropertyTweener* setDelay(double delay);

protected:
    void setTween(Tween* tween) override;
    void start() override;
    bool step(double& delta) override;
    void dropNode(Node* node) override;

private:
    friend class Tween;

    void setTargetNode(Node* node);

    ct::Function<TweenValue()> mGetter;
    ct::Function<void(const TweenValue&)> mSetter;
    Node* mTarget;
    bool mTargetAlive;
    TweenValue mInitialVal;
    TweenValue mBaseFinalVal;
    TweenValue mFinalVal;
    TweenValue mDeltaVal;
    double mDuration;
    TransitionType mTransType;
    EaseType mEaseType;
    ct::Function<float(float)> mCustomInterpolator;
    double mDelay;
    bool mDoContinue;
    bool mDoContinueDelayed;
    bool mRelative;
};

class IntervalTweener : public Tweener
{
public:
    IntervalTweener(TweenKey key, double time);

protected:
    bool step(double& delta) override;

private:
    double mDuration;
};

class CallbackTweener : public Tweener
{
public:
    CallbackTweener(TweenKey key, ct::Function<void()> callback);

    CallbackTweener* setDelay(double delay);

protected:
    bool step(double& delta) override;

private:
    ct::Function<void()> mCallback;
    double mDelay;
};

class MethodTweener : public Tweener
{
public:
    MethodTweener(TweenKey key, ct::Function<void(const TweenValue&)> callback,
            const TweenValue& from, const TweenValue& to, double duration);

    MethodTweener* setTrans(TransitionType trans);
    MethodTweener* setEase(EaseType ease);
    MethodTweener* setDelay(double delay);

protected:
    void setTween(Tween* tween) override;
    bool step(double& delta) override;

private:
    double mDuration;
    double mDelay;
    TransitionType mTransType;
    EaseType mEaseType;
    TweenValue mInitialVal;
    TweenValue mDeltaVal;
    TweenValue mFinalVal;
    ct::Function<void(const TweenValue&)> mCallback;
};

class Tween
{
public:
    explicit Tween(TweenKey key);
    Tween(const Tween&) = delete;
    Tween& operator=(const Tween&) = delete;

    PropertyTweener* tweenProperty(ct::Function<TweenValue()> getter,
            ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration);
    PropertyTweener* tweenPosition(Node3D* node, const Math::Vec3& to, double duration);
    PropertyTweener* tweenGlobalPosition(Node3D* node, const Math::Vec3& to, double duration);
    PropertyTweener* tweenRotation(Node3D* node, const Math::Quaternion& to, double duration);
    PropertyTweener* tweenScale(Node3D* node, const Math::Vec3& to, double duration);
    IntervalTweener* tweenInterval(double time);
    CallbackTweener* tweenCallback(ct::Function<void()> callback);
    MethodTweener* tweenMethod(ct::Function<void(const TweenValue&)> method, const TweenValue& from,
            const TweenValue& to, double duration);

    bool customStep(double delta);
    void stop();
    void pause();
    void play();
    void kill();

    bool hasTweeners() const;
    bool isPersistent() const;
    bool isRunning() const;
    bool isValid() const;
    void clear();

    Tween* bindNode(Node* node);
    Tween* setProcessMode(TweenProcessMode mode);
    TweenProcessMode getProcessMode() const;
    Tween* setPauseMode(TweenPauseMode mode);
    TweenPauseMode getPauseMode() const;

    Tween* setParallel(bool parallel);
    Tween* setLoops(int loops);
    int getLoopsLeft() const;
    Tween* setSpeedScale(float speed);
    Tween* setTrans(TransitionType trans);
    TransitionType getTrans() const;
    Tween* setEase(EaseType ease);
    EaseType getEase() const;

    Tween* parallel();
    Tween* chain();

    Node* getBoundNode() const;
    double getTotalElapsedTime() const;

    static float runEquation(TransitionType trans, EaseType ease, float t, float b, float c,
            float d);
    static TweenValue interpolateValue(const TweenValue& initial, const TweenValue& delta,
            double elapsed, double duration, TransitionType trans, EaseType ease);

    ct::Function<void()> finished;
    ct::Function<void(int)> stepFinished;
    ct::Function<void(int)> loopFinished;

private:
    friend class SceneTree;

    void startTweeners();
    void stopInternal(bool reset);
    void append(Tweener* tweener);
    void releaseTweeners();
    PropertyTweener* tweenNodeProperty(Node3D* node, ct::Function<TweenValue()> getter,
            ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration);
    bool step(double delta);
    bool canProcess(bool treePaused) const;
    void dropNode(Node* node);
    ~Tween();

    TweenProcessMode mProcessMode;
    TweenPauseMode mPauseMode;
    TransitionType mDefaultTransition;
    EaseType mDefaultEase;
    Node* mBoundNode;
    ct::Vector<ct::Vector<Tweener*>> mTweeners;
    double mTotalTime;
    int mCurrentStep;
    int mLoops;
    int mLoopsDone;
    float mSpeedScale;
    bool mIsBound;
    bool mStarted;
    bool mRunning;
    bool mInStep;
    bool mClearPending;
    bool mDead;
    bool mValid;
    bool mDefaultParallel;
    bool mParallelEnabled;
    bool mPersistent;
};

} // namespace prisma3d
