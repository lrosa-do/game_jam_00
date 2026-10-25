#ifndef KINETIX_AI_STATE_H
#define KINETIX_AI_STATE_H

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace kinetix::AI
{

class Action;
class Transition;

class State
{
public:
    explicit State(const ct::String& name) : mName(name)
    {
    }
    State(float initialValue, const ct::String& name)
        : mValue(initialValue), mInitialValue(initialValue), mName(name)
    {
    }
    virtual ~State();

    float value() const
    {
        return mValue;
    }
    float initialValue() const
    {
        return mInitialValue;
    }
    void setValue(float value)
    {
        mValue = value;
    }
    void setInitialValue(float value)
    {
        mInitialValue = value;
    }

    // Lifecycle. reset() restores the value and re-enters the state.
    void reset();
    void enter();
    void iterate();
    void exit();

    const ct::String& name() const
    {
        return mName;
    }
    void setName(const ct::String& name)
    {
        mName = name;
    }

    // Ownership: the State takes ownership of actions and transitions passed
    // here and deletes them in its destructor.
    void addAction(Action* action)
    {
        mActions.push_back(action);
    }
    void addEnterAction(Action* action)
    {
        mEnterActions.push_back(action);
    }
    void addExitAction(Action* action)
    {
        mExitActions.push_back(action);
    }
    void addTransition(Transition* transition)
    {
        mTransitions.push_back(transition);
    }

    ct::Vector<Action*>& actions()
    {
        return mActions;
    }
    const ct::Vector<Action*>& actions() const
    {
        return mActions;
    }
    ct::Vector<Action*>& enterActions()
    {
        return mEnterActions;
    }
    const ct::Vector<Action*>& enterActions() const
    {
        return mEnterActions;
    }
    ct::Vector<Action*>& exitActions()
    {
        return mExitActions;
    }
    const ct::Vector<Action*>& exitActions() const
    {
        return mExitActions;
    }
    ct::Vector<Transition*>& transitions()
    {
        return mTransitions;
    }
    const ct::Vector<Transition*>& transitions() const
    {
        return mTransitions;
    }

protected:
    static void runActions(ct::Vector<Action*>& actions);

    float mValue = 0.0f;
    float mInitialValue = 0.0f;
    ct::Vector<Transition*> mTransitions;
    ct::Vector<Action*> mActions;
    ct::Vector<Action*> mEnterActions;
    ct::Vector<Action*> mExitActions;
    ct::String mName;
};

}

#endif
