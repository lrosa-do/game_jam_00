#include "prisma3d/scene/Tween.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"

#include "EasingEquations.h"
#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const double kCmpEpsilon = 0.00001;

typedef float (*Interpolator)(float t, float b, float c, float d);

const Interpolator kInterpolators[static_cast<int>(TransitionType::Max)][static_cast<int>(
        EaseType::Max)] = {
    { &easing::Linear::in, &easing::Linear::in, &easing::Linear::in, &easing::Linear::in },
    { &easing::Sine::in, &easing::Sine::out, &easing::Sine::inOut, &easing::Sine::outIn },
    { &easing::Quint::in, &easing::Quint::out, &easing::Quint::inOut, &easing::Quint::outIn },
    { &easing::Quart::in, &easing::Quart::out, &easing::Quart::inOut, &easing::Quart::outIn },
    { &easing::Quad::in, &easing::Quad::out, &easing::Quad::inOut, &easing::Quad::outIn },
    { &easing::Expo::in, &easing::Expo::out, &easing::Expo::inOut, &easing::Expo::outIn },
    { &easing::Elastic::in, &easing::Elastic::out, &easing::Elastic::inOut,
        &easing::Elastic::outIn },
    { &easing::Cubic::in, &easing::Cubic::out, &easing::Cubic::inOut, &easing::Cubic::outIn },
    { &easing::Circ::in, &easing::Circ::out, &easing::Circ::inOut, &easing::Circ::outIn },
    { &easing::Bounce::in, &easing::Bounce::out, &easing::Bounce::inOut, &easing::Bounce::outIn },
    { &easing::Back::in, &easing::Back::out, &easing::Back::inOut, &easing::Back::outIn },
    { &easing::Spring::in, &easing::Spring::out, &easing::Spring::inOut, &easing::Spring::outIn },
};

bool isZeroApprox(double value)
{
    return fabs(value) < kCmpEpsilon;
}

bool isEqualApprox(double a, double b)
{
    if (a == b) return true;
    double tolerance = kCmpEpsilon * fabs(a);
    if (tolerance < kCmpEpsilon) tolerance = kCmpEpsilon;
    return fabs(a - b) < tolerance;
}

float lerpFloat(float from, float to, float weight)
{
    return from + (to - from) * weight;
}

double lerpDouble(double from, double to, double weight)
{
    return from + (to - from) * weight;
}

Math::Quaternion slerp(const Math::Quaternion& from, const Math::Quaternion& to, float weight)
{
    Math::Quaternion to1;
    float omega;
    float cosom;
    float sinom;
    float scale0;
    float scale1;

    cosom = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

    if (cosom < 0.0f)
    {
        cosom = -cosom;
        to1 = -to;
    }
    else
        to1 = to;

    if ((1.0f - cosom) > static_cast<float>(kCmpEpsilon))
    {
        omega = cosom < -1.0f ? static_cast<float>(easing::math::kPi)
                              : (cosom > 1.0f ? 0.0f : acosf(cosom));
        sinom = easing::math::sin(omega);
        scale0 = easing::math::sin((1.0 - weight) * omega) / sinom;
        scale1 = easing::math::sin(weight * omega) / sinom;
    }
    else
    {
        scale0 = 1.0f - weight;
        scale1 = weight;
    }

    return Math::Quaternion(scale0 * from.x + scale1 * to1.x, scale0 * from.y + scale1 * to1.y,
            scale0 * from.z + scale1 * to1.z, scale0 * from.w + scale1 * to1.w);
}

bool validateTypeMatch(const TweenValue& from, const TweenValue& to)
{
    if (from.type() != to.type())
    {
        log_error("tween: type mismatch between initial and final value");
        return false;
    }
    return true;
}

TweenValue addValue(const TweenValue& a, const TweenValue& b)
{
    if (a.type() != b.type()) return a;

    switch (a.type())
    {
        case TweenValue::Type::Float:
            return TweenValue(a.asFloat() + b.asFloat());
        case TweenValue::Type::Vec3:
            return TweenValue(a.asVec3() + b.asVec3());
        case TweenValue::Type::Quaternion:
            return TweenValue(a.asQuaternion() * b.asQuaternion());
        case TweenValue::Type::Color:
            return TweenValue(a.asColor() + b.asColor());
    }
    return a;
}

TweenValue subtractValue(const TweenValue& a, const TweenValue& b)
{
    if (a.type() != b.type()) return a;

    switch (a.type())
    {
        case TweenValue::Type::Float:
            return TweenValue(a.asFloat() - b.asFloat());
        case TweenValue::Type::Vec3:
            return TweenValue(a.asVec3() - b.asVec3());
        case TweenValue::Type::Quaternion:
            return TweenValue(b.asQuaternion().Conjugate() * a.asQuaternion());
        case TweenValue::Type::Color:
            return TweenValue(a.asColor() - b.asColor());
    }
    return a;
}

TweenValue interpolateVariant(const TweenValue& a, const TweenValue& b, float c)
{
    if (a.type() != b.type()) return a;

    switch (a.type())
    {
        case TweenValue::Type::Float:
            return TweenValue(lerpDouble(a.asFloat(), b.asFloat(), static_cast<double>(c)));
        case TweenValue::Type::Vec3:
        {
            const Math::Vec3 from = a.asVec3();
            const Math::Vec3 to = b.asVec3();
            return TweenValue(Math::Vec3(lerpFloat(from.x, to.x, c), lerpFloat(from.y, to.y, c),
                    lerpFloat(from.z, to.z, c)));
        }
        case TweenValue::Type::Quaternion:
            return TweenValue(slerp(a.asQuaternion(), b.asQuaternion(), c));
        case TweenValue::Type::Color:
        {
            const Math::Vec4 from = a.asColor();
            const Math::Vec4 to = b.asColor();
            return TweenValue(Math::Vec4(lerpFloat(from.x, to.x, c), lerpFloat(from.y, to.y, c),
                    lerpFloat(from.z, to.z, c), lerpFloat(from.w, to.w, c)));
        }
    }
    return a;
}

} // namespace

TweenValue::TweenValue()
        : mType(Type::Float),
          mFloat(0.0),
          mVector(0.0f, 0.0f, 0.0f, 0.0f)
{
}

TweenValue::TweenValue(double value)
        : mType(Type::Float),
          mFloat(value),
          mVector(0.0f, 0.0f, 0.0f, 0.0f)
{
}

TweenValue::TweenValue(float value)
        : mType(Type::Float),
          mFloat(static_cast<double>(value)),
          mVector(0.0f, 0.0f, 0.0f, 0.0f)
{
}

TweenValue::TweenValue(int value)
        : mType(Type::Float),
          mFloat(static_cast<double>(value)),
          mVector(0.0f, 0.0f, 0.0f, 0.0f)
{
}

TweenValue::TweenValue(const Math::Vec3& value)
        : mType(Type::Vec3),
          mFloat(0.0),
          mVector(value.x, value.y, value.z, 0.0f)
{
}

TweenValue::TweenValue(const Math::Quaternion& value)
        : mType(Type::Quaternion),
          mFloat(0.0),
          mVector(value.x, value.y, value.z, value.w)
{
}

TweenValue::TweenValue(const Math::Vec4& color)
        : mType(Type::Color),
          mFloat(0.0),
          mVector(color)
{
}

TweenValue::Type TweenValue::type() const
{
    return mType;
}

double TweenValue::asFloat() const
{
    return mFloat;
}

Math::Vec3 TweenValue::asVec3() const
{
    return Math::Vec3(mVector.x, mVector.y, mVector.z);
}

Math::Quaternion TweenValue::asQuaternion() const
{
    return Math::Quaternion(mVector.x, mVector.y, mVector.z, mVector.w);
}

Math::Vec4 TweenValue::asColor() const
{
    return mVector;
}

Tweener::Tweener()
        : finished(),
          mElapsedTime(0.0),
          mFinished(false),
          mTween(nullptr)
{
}

Tweener::~Tweener() = default;

void Tweener::setTween(Tween* tween)
{
    mTween = tween;
}

void Tweener::start()
{
    mElapsedTime = 0.0;
    mFinished = false;
}

void Tweener::dropNode(Node* node)
{
    (void) node;
}

Tween* Tweener::getTween() const
{
    return mTween;
}

void Tweener::finish()
{
    mFinished = true;
    if (finished) finished();
}

PropertyTweener::PropertyTweener(TweenKey key, ct::Function<TweenValue()> getter,
        ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration)
        : Tweener(),
          mGetter(getter),
          mSetter(setter),
          mTarget(nullptr),
          mTargetAlive(true),
          mInitialVal(),
          mBaseFinalVal(to),
          mFinalVal(to),
          mDeltaVal(),
          mDuration(duration),
          mTransType(TransitionType::Max),
          mEaseType(EaseType::Max),
          mCustomInterpolator(),
          mDelay(0.0),
          mDoContinue(true),
          mDoContinueDelayed(false),
          mRelative(false)
{
    (void) key;
    mInitialVal = mGetter();
}

PropertyTweener* PropertyTweener::from(const TweenValue& value)
{
    if (!getTween())
    {
        log_error("tween: property tweener has no tween");
        return nullptr;
    }

    if (!validateTypeMatch(mFinalVal, value)) return nullptr;

    mInitialVal = value;
    mDoContinue = false;
    return this;
}

PropertyTweener* PropertyTweener::fromCurrent()
{
    mDoContinue = false;
    return this;
}

PropertyTweener* PropertyTweener::asRelative()
{
    mRelative = true;
    return this;
}

PropertyTweener* PropertyTweener::setTrans(TransitionType trans)
{
    mTransType = trans;
    return this;
}

PropertyTweener* PropertyTweener::setEase(EaseType ease)
{
    mEaseType = ease;
    return this;
}

PropertyTweener* PropertyTweener::setCustomInterpolator(ct::Function<float(float)> interpolator)
{
    mCustomInterpolator = interpolator;
    return this;
}

PropertyTweener* PropertyTweener::setDelay(double delay)
{
    mDelay = delay;
    return this;
}

void PropertyTweener::setTargetNode(Node* node)
{
    mTarget = node;
}

void PropertyTweener::dropNode(Node* node)
{
    if (node && mTarget == node)
    {
        mTarget = nullptr;
        mTargetAlive = false;
    }
}

void PropertyTweener::start()
{
    Tweener::start();

    if (!mTargetAlive) return;

    if (mDoContinue)
    {
        if (isZeroApprox(mDelay)) mInitialVal = mGetter();
        else
            mDoContinueDelayed = true;
    }

    if (mRelative) mFinalVal = addValue(mInitialVal, mBaseFinalVal);

    mDeltaVal = subtractValue(mFinalVal, mInitialVal);
}

bool PropertyTweener::step(double& delta)
{
    if (mFinished) return false;

    if (!mTargetAlive)
    {
        finish();
        return false;
    }
    mElapsedTime += delta;

    if (mElapsedTime < mDelay)
    {
        delta = 0;
        return true;
    }
    else if (mDoContinueDelayed && !isZeroApprox(mDelay))
    {
        mInitialVal = mGetter();
        mDeltaVal = subtractValue(mFinalVal, mInitialVal);
        mDoContinueDelayed = false;
    }

    double time = mElapsedTime - mDelay;
    if (mDuration < time) time = mDuration;
    if (time < mDuration)
    {
        if (mCustomInterpolator)
        {
            const TweenValue t = Tween::interpolateValue(TweenValue(0.0), TweenValue(1.0), time,
                    mDuration, mTransType, mEaseType);
            double result =
                    static_cast<double>(mCustomInterpolator(static_cast<float>(t.asFloat())));
            mSetter(interpolateVariant(mInitialVal, mFinalVal, static_cast<float>(result)));
        }
        else
        {
            mSetter(Tween::interpolateValue(mInitialVal, mDeltaVal, time, mDuration, mTransType,
                    mEaseType));
        }
        delta = 0;
        return true;
    }
    else
    {
        if (mCustomInterpolator)
        {
            double finalT = static_cast<double>(mCustomInterpolator(1.0f));
            mSetter(interpolateVariant(mInitialVal, mFinalVal, static_cast<float>(finalT)));
        }
        else
            mSetter(mFinalVal);
        delta = mElapsedTime - mDelay - mDuration;
        finish();
        return false;
    }
}

void PropertyTweener::setTween(Tween* tween)
{
    Tweener::setTween(tween);
    if (mTransType == TransitionType::Max) mTransType = tween->getTrans();
    if (mEaseType == EaseType::Max) mEaseType = tween->getEase();
}

IntervalTweener::IntervalTweener(TweenKey key, double time)
        : Tweener(),
          mDuration(time)
{
    (void) key;
}

bool IntervalTweener::step(double& delta)
{
    if (mFinished) return false;

    mElapsedTime += delta;

    if (mElapsedTime < mDuration)
    {
        delta = 0;
        return true;
    }
    else
    {
        delta = mElapsedTime - mDuration;
        finish();
        return false;
    }
}

CallbackTweener::CallbackTweener(TweenKey key, ct::Function<void()> callback)
        : Tweener(),
          mCallback(callback),
          mDelay(0.0)
{
    (void) key;
}

CallbackTweener* CallbackTweener::setDelay(double delay)
{
    mDelay = delay;
    return this;
}

bool CallbackTweener::step(double& delta)
{
    if (mFinished) return false;

    if (!mCallback)
    {
        finish();
        return false;
    }

    mElapsedTime += delta;
    if (mElapsedTime >= mDelay)
    {
        mCallback();

        delta = mElapsedTime - mDelay;
        finish();
        return false;
    }

    delta = 0;
    return true;
}

MethodTweener::MethodTweener(TweenKey key, ct::Function<void(const TweenValue&)> callback,
        const TweenValue& from, const TweenValue& to, double duration)
        : Tweener(),
          mDuration(duration),
          mDelay(0.0),
          mTransType(TransitionType::Max),
          mEaseType(EaseType::Max),
          mInitialVal(from),
          mDeltaVal(),
          mFinalVal(to),
          mCallback(callback)
{
    (void) key;
    mDeltaVal = subtractValue(to, from);
}

MethodTweener* MethodTweener::setTrans(TransitionType trans)
{
    mTransType = trans;
    return this;
}

MethodTweener* MethodTweener::setEase(EaseType ease)
{
    mEaseType = ease;
    return this;
}

MethodTweener* MethodTweener::setDelay(double delay)
{
    mDelay = delay;
    return this;
}

bool MethodTweener::step(double& delta)
{
    if (mFinished) return false;

    if (!mCallback)
    {
        finish();
        return false;
    }

    mElapsedTime += delta;

    if (mElapsedTime < mDelay)
    {
        delta = 0;
        return true;
    }

    TweenValue currentVal;
    double time = mElapsedTime - mDelay;
    if (mDuration < time) time = mDuration;
    if (time < mDuration)
        currentVal = Tween::interpolateValue(mInitialVal, mDeltaVal, time, mDuration, mTransType,
                mEaseType);
    else
        currentVal = mFinalVal;

    mCallback(currentVal);

    if (time < mDuration)
    {
        delta = 0;
        return true;
    }
    else
    {
        delta = mElapsedTime - mDelay - mDuration;
        finish();
        return false;
    }
}

void MethodTweener::setTween(Tween* tween)
{
    Tweener::setTween(tween);
    if (mTransType == TransitionType::Max) mTransType = tween->getTrans();
    if (mEaseType == EaseType::Max) mEaseType = tween->getEase();
}

Tween::Tween(TweenKey key)
        : finished(),
          stepFinished(),
          loopFinished(),
          mProcessMode(TweenProcessMode::Idle),
          mPauseMode(TweenPauseMode::Bound),
          mDefaultTransition(TransitionType::Linear),
          mDefaultEase(EaseType::InOut),
          mBoundNode(nullptr),
          mTweeners(),
          mTotalTime(0.0),
          mCurrentStep(-1),
          mLoops(1),
          mLoopsDone(0),
          mSpeedScale(1.0f),
          mIsBound(false),
          mStarted(false),
          mRunning(true),
          mInStep(false),
          mClearPending(false),
          mDead(false),
          mValid(true),
          mDefaultParallel(false),
          mParallelEnabled(false),
          mPersistent(false)
{
    (void) key;
}

Tween::~Tween()
{
    releaseTweeners();
}

void Tween::releaseTweeners()
{
    for (std::size_t i = 0; i < mTweeners.size(); ++i)
        for (std::size_t j = 0; j < mTweeners[i].size(); ++j) delete mTweeners[i][j];
    mTweeners.clear();
    mClearPending = false;
}

void Tween::startTweeners()
{
    if (mTweeners.empty())
    {
        mDead = true;
        log_error("tween: without commands, aborting");
        return;
    }

    ct::Vector<Tweener*>& list = mTweeners[static_cast<std::size_t>(mCurrentStep)];
    for (std::size_t i = 0; i < list.size(); ++i) list[i]->start();
}

void Tween::stopInternal(bool reset)
{
    mRunning = false;
    if (reset)
    {
        mStarted = false;
        mDead = false;
        mTotalTime = 0.0;
    }
}

PropertyTweener* Tween::tweenProperty(ct::Function<TweenValue()> getter,
        ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration)
{
    if (!mValid)
    {
        log_error("tween: invalid, either finished or created outside the scene tree");
        return nullptr;
    }
    if (mStarted)
    {
        log_error("tween: can't append to a tween that has started, use stop() first");
        return nullptr;
    }
    if (!getter || !setter)
    {
        log_error("tween: a property tweener needs a getter and a setter");
        return nullptr;
    }

    if (!validateTypeMatch(getter(), to)) return nullptr;

    PropertyTweener* tweener = new PropertyTweener(TweenKey(), getter, setter, to, duration);
    append(tweener);
    return tweener;
}

PropertyTweener* Tween::tweenNodeProperty(Node3D* node, ct::Function<TweenValue()> getter,
        ct::Function<void(const TweenValue&)> setter, const TweenValue& to, double duration)
{
    if (!node)
    {
        log_error("tween: the target node is null");
        return nullptr;
    }

    PropertyTweener* tweener = tweenProperty(getter, setter, to, duration);
    if (tweener) tweener->setTargetNode(node);
    return tweener;
}

PropertyTweener* Tween::tweenPosition(Node3D* node, const Math::Vec3& to, double duration)
{
    return tweenNodeProperty(
            node, [node]() { return TweenValue(node->getPosition()); },
            [node](const TweenValue& value) { node->setPosition(value.asVec3()); }, TweenValue(to),
            duration);
}

PropertyTweener* Tween::tweenGlobalPosition(Node3D* node, const Math::Vec3& to, double duration)
{
    return tweenNodeProperty(
            node, [node]() { return TweenValue(node->getGlobalPosition()); },
            [node](const TweenValue& value) { node->setGlobalPosition(value.asVec3()); },
            TweenValue(to), duration);
}

PropertyTweener* Tween::tweenRotation(Node3D* node, const Math::Quaternion& to, double duration)
{
    return tweenNodeProperty(
            node, [node]() { return TweenValue(node->getQuaternion()); },
            [node](const TweenValue& value) { node->setQuaternion(value.asQuaternion()); },
            TweenValue(to), duration);
}

PropertyTweener* Tween::tweenScale(Node3D* node, const Math::Vec3& to, double duration)
{
    return tweenNodeProperty(
            node, [node]() { return TweenValue(node->getScale()); }, [node](const TweenValue& value)
            { node->setScale(value.asVec3()); }, TweenValue(to), duration);
}

IntervalTweener* Tween::tweenInterval(double time)
{
    if (!mValid)
    {
        log_error("tween: invalid, either finished or created outside the scene tree");
        return nullptr;
    }
    if (mStarted)
    {
        log_error("tween: can't append to a tween that has started, use stop() first");
        return nullptr;
    }

    IntervalTweener* tweener = new IntervalTweener(TweenKey(), time);
    append(tweener);
    return tweener;
}

CallbackTweener* Tween::tweenCallback(ct::Function<void()> callback)
{
    if (!mValid)
    {
        log_error("tween: invalid, either finished or created outside the scene tree");
        return nullptr;
    }
    if (mStarted)
    {
        log_error("tween: can't append to a tween that has started, use stop() first");
        return nullptr;
    }

    CallbackTweener* tweener = new CallbackTweener(TweenKey(), callback);
    append(tweener);
    return tweener;
}

MethodTweener* Tween::tweenMethod(ct::Function<void(const TweenValue&)> method,
        const TweenValue& from, const TweenValue& to, double duration)
{
    if (!mValid)
    {
        log_error("tween: invalid, either finished or created outside the scene tree");
        return nullptr;
    }
    if (mStarted)
    {
        log_error("tween: can't append to a tween that has started, use stop() first");
        return nullptr;
    }

    if (!validateTypeMatch(from, to)) return nullptr;

    MethodTweener* tweener = new MethodTweener(TweenKey(), method, from, to, duration);
    append(tweener);
    return tweener;
}

void Tween::append(Tweener* tweener)
{
    tweener->setTween(this);

    if (mParallelEnabled)
    {
        if (mCurrentStep < 0) mCurrentStep = 0;
    }
    else
        mCurrentStep++;
    mParallelEnabled = mDefaultParallel;

    mTweeners.resize(static_cast<std::size_t>(mCurrentStep) + 1);
    mTweeners[static_cast<std::size_t>(mCurrentStep)].push_back(tweener);
}

void Tween::stop()
{
    stopInternal(true);
}

void Tween::pause()
{
    stopInternal(false);
}

void Tween::play()
{
    if (!mValid)
    {
        log_error("tween: invalid, either finished or created outside the scene tree");
        return;
    }
    if (mDead)
    {
        log_error("tween: can't play a finished tween, use stop() first to reset its state");
        return;
    }
    mRunning = true;
}

void Tween::kill()
{
    mRunning = false;
    mValid = false;
    mDead = true;
}

bool Tween::hasTweeners() const
{
    return !mTweeners.empty();
}

bool Tween::isPersistent() const
{
    return mPersistent;
}

bool Tween::isRunning() const
{
    return mRunning;
}

bool Tween::isValid() const
{
    return mValid;
}

void Tween::clear()
{
    mValid = false;
    if (mInStep) mClearPending = true;
    else
        releaseTweeners();
}

Tween* Tween::bindNode(Node* node)
{
    if (!node)
    {
        log_error("tween: cannot bind a null node");
        return this;
    }

    mBoundNode = node;
    mIsBound = true;
    return this;
}

Tween* Tween::setProcessMode(TweenProcessMode mode)
{
    mProcessMode = mode;
    return this;
}

TweenProcessMode Tween::getProcessMode() const
{
    return mProcessMode;
}

Tween* Tween::setPauseMode(TweenPauseMode mode)
{
    mPauseMode = mode;
    return this;
}

TweenPauseMode Tween::getPauseMode() const
{
    return mPauseMode;
}

Tween* Tween::setParallel(bool parallel)
{
    mDefaultParallel = parallel;
    mParallelEnabled = parallel;
    return this;
}

Tween* Tween::setLoops(int loops)
{
    mLoops = loops;
    return this;
}

int Tween::getLoopsLeft() const
{
    if (mLoops <= 0) return -1;
    return mLoops - mLoopsDone;
}

Tween* Tween::setSpeedScale(float speed)
{
    mSpeedScale = speed;
    return this;
}

Tween* Tween::setTrans(TransitionType trans)
{
    mDefaultTransition = trans;
    return this;
}

TransitionType Tween::getTrans() const
{
    return mDefaultTransition;
}

Tween* Tween::setEase(EaseType ease)
{
    mDefaultEase = ease;
    return this;
}

EaseType Tween::getEase() const
{
    return mDefaultEase;
}

Tween* Tween::parallel()
{
    mParallelEnabled = true;
    return this;
}

Tween* Tween::chain()
{
    mParallelEnabled = false;
    return this;
}

bool Tween::customStep(double delta)
{
    if (mInStep)
    {
        log_error("tween: can't call customStep() during another tween step");
        return true;
    }

    bool r = mRunning;
    mRunning = true;
    bool ret = step(delta);
    mRunning = mRunning && r;
    return ret;
}

bool Tween::step(double delta)
{
    if (mDead) return false;

    if (mIsBound)
    {
        Node* node = getBoundNode();
        if (node)
        {
            if (!node->isInsideTree()) return true;
        }
        else
            return false;
    }

    if (!mRunning) return true;
    mInStep = true;

    if (!mStarted)
    {
        if (mTweeners.empty())
        {
            Node* node = getBoundNode();
            if (node)
                log_error("tween (bound to %s): started with no tweeners", node->getName().c_str());
            else
                log_error("tween: started with no tweeners");
            mInStep = false;
            return false;
        }
        mCurrentStep = 0;
        mLoopsDone = 0;
        mTotalTime = 0.0;
        startTweeners();
        mStarted = true;
    }

    double remDelta = delta * mSpeedScale;
    bool stepActive = false;
    mTotalTime += remDelta;

    double initialDelta = remDelta;
    bool potentialInfinite = false;

    while (mRunning && remDelta > 0)
    {
        double stepDelta = remDelta;
        stepActive = false;

        ct::Vector<Tweener*>& current = mTweeners[static_cast<std::size_t>(mCurrentStep)];
        for (std::size_t i = 0; i < current.size(); ++i)
        {
            double tempDelta = remDelta;
            stepActive = current[i]->step(tempDelta) || stepActive;
            if (tempDelta < stepDelta) stepDelta = tempDelta;
        }

        remDelta = stepDelta;

        if (!stepActive)
        {
            if (stepFinished) stepFinished(mCurrentStep);
            mCurrentStep++;

            if (mCurrentStep == static_cast<int>(mTweeners.size()))
            {
                mLoopsDone++;
                if (mLoopsDone == mLoops)
                {
                    mRunning = false;
                    if (mPersistent) mStarted = false;
                    else
                        mDead = true;
                    if (finished) finished();
                    break;
                }
                else
                {
                    if (loopFinished) loopFinished(mLoopsDone);
                    mCurrentStep = 0;
                    startTweeners();
                    if (mLoops <= 0 && isEqualApprox(remDelta, initialDelta))
                    {
                        if (!potentialInfinite) potentialInfinite = true;
                        else
                        {
                            mInStep = false;
                            if (mClearPending) releaseTweeners();
                            log_error("tween: infinite loop detected, check setLoops()");
                            return false;
                        }
                    }
                }
            }
            else
                startTweeners();
        }
    }
    mInStep = false;
    if (mClearPending) releaseTweeners();
    return true;
}

bool Tween::canProcess(bool treePaused) const
{
    if (mIsBound && mPauseMode == TweenPauseMode::Bound)
    {
        Node* node = getBoundNode();
        if (node) return node->isInsideTree() && !treePaused;
    }

    return !treePaused || mPauseMode == TweenPauseMode::Process;
}

void Tween::dropNode(Node* node)
{
    if (!node) return;

    if (mIsBound && mBoundNode == node) mBoundNode = nullptr;

    for (std::size_t i = 0; i < mTweeners.size(); ++i)
        for (std::size_t j = 0; j < mTweeners[i].size(); ++j) mTweeners[i][j]->dropNode(node);
}

Node* Tween::getBoundNode() const
{
    return mIsBound ? mBoundNode : nullptr;
}

double Tween::getTotalElapsedTime() const
{
    return mTotalTime;
}

float Tween::runEquation(TransitionType trans, EaseType ease, float t, float b, float c, float d)
{
    if (d == 0)
    {
        return b + c;
    }

    Interpolator func = kInterpolators[static_cast<int>(trans)][static_cast<int>(ease)];
    return func(t, b, c, d);
}

TweenValue Tween::interpolateValue(const TweenValue& initial, const TweenValue& delta,
        double elapsed, double duration, TransitionType trans, EaseType ease)
{
    if (trans >= TransitionType::Max)
    {
        log_error("tween: transition type out of range");
        return TweenValue();
    }
    if (ease >= EaseType::Max)
    {
        log_error("tween: ease type out of range");
        return TweenValue();
    }

    TweenValue ret = addValue(initial, delta);
    ret = interpolateVariant(initial, ret,
            runEquation(trans, ease, static_cast<float>(elapsed), 0.0f, 1.0f,
                    static_cast<float>(duration)));
    return ret;
}

} // namespace prisma3d
