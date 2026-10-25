#ifndef KINETIX_AI_TRANSITION_H
#define KINETIX_AI_TRANSITION_H

#include "State.h"

#include <ct/function.hpp>
#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

class State;

class Transition
{
public:
    Transition(State* source, State* target) : mSource(source), mTarget(target)
    {
    }
    virtual ~Transition() = default;

    virtual bool shouldTransition()
    {
        return false;
    }

    State& source() const
    {
        return *mSource;
    }
    State& target() const
    {
        return *mTarget;
    }
    State* sourcePtr() const
    {
        return mSource;
    }
    State* targetPtr() const
    {
        return mTarget;
    }

    virtual const char* label() const
    {
        return "Base Transition";
    }

protected:
    State* mSource; // non-owning; owned by the StateMachine
    State* mTarget; // non-owning; owned by the StateMachine
};

class Comparitor
{
public:
    virtual ~Comparitor() = default;
    virtual bool compare(float lhs, float rhs) = 0;
    virtual const char* label() const = 0;
};

class GreaterThanComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs > rhs;
    }
    const char* label() const override
    {
        return ">";
    }
};

class GreaterOrEqualComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs >= rhs;
    }
    const char* label() const override
    {
        return ">=";
    }
};

class LessThanComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs < rhs;
    }
    const char* label() const override
    {
        return "<";
    }
};

class LessOrEqualComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs <= rhs;
    }
    const char* label() const override
    {
        return "<=";
    }
};

class EqualComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs == rhs;
    }
    const char* label() const override
    {
        return "==";
    }
};

class NotEqualComparitor final : public Comparitor
{
public:
    bool compare(float lhs, float rhs) override
    {
        return lhs != rhs;
    }
    const char* label() const override
    {
        return "!=";
    }
};

// Fires when source().value() compares against a fixed threshold.
class ComparitorTransition final : public Transition
{
public:
    ComparitorTransition(State* source, State* target, float threshold, Comparitor* func)
        : Transition(source, target), mThreshold(threshold), mFunc(func)
    {
    }
    ~ComparitorTransition() override
    {
        delete mFunc;
    }

    bool shouldTransition() override
    {
        return mFunc->compare(source().value(), mThreshold);
    }

    float threshold() const
    {
        return mThreshold;
    }
    void setThreshold(float threshold)
    {
        mThreshold = threshold;
    }

    const char* label() const override
    {
        return mFunc->label();
    }

private:
    float mThreshold;
    Comparitor* mFunc; // owned by this transition
};

// Fires when the user functor returns true; capture the owning entity in the lambda.
class CallbackTransition final : public Transition
{
public:
    using Callback = ct::Function<bool(State&)>;

    CallbackTransition(State* source, State* target, Callback callback)
        : Transition(source, target), mCallback(kinetix::Move(callback))
    {
    }

    bool shouldTransition() override
    {
        return mCallback ? mCallback(source()) : false;
    }
    const char* label() const override
    {
        return "Callback Transition";
    }

private:
    Callback mCallback;
};

}

#endif
