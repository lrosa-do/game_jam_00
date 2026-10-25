#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "StateMachine.h"

#include "Action.h"
#include "State.h"
#include "Transition.h"

#include <stdio.h>

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

StateMachine::~StateMachine()
{
    for (State* state : mStates)
        delete state;
    mStates.clear();
    mCurrentState = nullptr;

    mCallbackDepth = 0;
    releaseRemoved();
}

void StateMachine::iterate()
{
    ++mCallbackDepth;
    step();
    --mCallbackDepth;
    releaseRemoved();
}

void StateMachine::step()
{
    // Every call below is a user callback that can reach back in here: setCurrentState(),
    // removeState() (deletes the State and every transition pointing at it), or anything destroying
    // what the callbacks captured. The state is pinned before each one and rechecked after; if it
    // moved, this tick is over.
    State* state = mCurrentState;
    if (!state)
        return;

    state->iterate();
    if (mCurrentState != state)
        return;

    // By index, with size() re-read every turn: a callback may erase transitions from this very
    // vector (removeState() does that to any transition aimed at the removed state).
    ct::Vector<Transition*>& transitions = state->transitions();
    for (usize i = 0; i < transitions.size(); ++i)
    {
        Transition* transition = transitions[i];
        if (!transition)
            continue;
        if (!transition->shouldTransition())
        {
            if (mCurrentState != state)
                return;
            continue;
        }

        // Read the target before exit() runs: exit() can remove states, and
        // removeState() deletes the transitions that name them.
        State* target = transition->targetPtr();
        state->exit();
        if (mCurrentState != state)
            return;
        // A callback may have removed the target meanwhile: it is still
        // allocated (deletion waits for the end of this tick) but must not
        // become current.
        if (!target || !contains(target))
            return;
        mCurrentState = target;
        target->enter();
        return;
    }
}

void StateMachine::addState(State* state)
{
    if (!state)
        return;
    if (kinetix::Find(mStates.begin(), mStates.end(), state) != mStates.end())
        return;

    if (!mCurrentState)
        mCurrentState = state;
    mStates.push_back(state);
}

void StateMachine::removeState(State* state)
{
    auto it = kinetix::Find(mStates.begin(), mStates.end(), state);
    if (it == mStates.end())
        return;

    mStates.erase(it);
    if (mCurrentState == state)
        mCurrentState = nullptr;

    // A transition in any other state can still point at `state` through targetPtr()/sourcePtr();
    // it would dangle once `state` is deleted, and iterate() could install it as mCurrentState or
    // toDot() dereference it. `state`'s own outgoing transitions die with it. Inside a callback the
    // pruned transitions and the state are only set aside: the running callback may be one of them,
    // or an action owned by `state`, or iterate() may still be walking its lists.
    for (State* other : mStates)
    {
        ct::Vector<Transition*>& transitions = other->transitions();
        for (usize i = transitions.size(); i > 0; --i)
        {
            Transition* transition = transitions[i - 1];
            if (transition->targetPtr() == state || transition->sourcePtr() == state)
            {
                transitions.erase(transitions.begin() + (i - 1));
                mRemovedTransitions.push_back(transition);
            }
        }
    }

    mRemovedStates.push_back(state);
    releaseRemoved();
}

bool StateMachine::contains(const State* state) const
{
    return kinetix::Find(mStates.begin(), mStates.end(), state) != mStates.end();
}

void StateMachine::releaseRemoved()
{
    if (mCallbackDepth != 0)
        return;

    // Swapped out first: a destructor below is user code too and could
    // remove further states.
    ct::Vector<Transition*> transitions = kinetix::Move(mRemovedTransitions);
    mRemovedTransitions.clear();
    StateList states = kinetix::Move(mRemovedStates);
    mRemovedStates.clear();

    for (Transition* transition : transitions)
        delete transition;
    for (State* state : states)
        delete state;
}

State* StateMachine::findState(const ct::String& name) const
{
    for (State* state : mStates)
    {
        if (state->name() == name)
            return state;
    }
    return nullptr;
}

void StateMachine::reset()
{
    // Jump to the first state registered and re-enter it.
    if (!mStates.empty())
        mCurrentState = mStates.front();
    if (mCurrentState)
    {
        ++mCallbackDepth;
        mCurrentState->reset();
        --mCallbackDepth;
        releaseRemoved();
    }
}

State* StateMachine::constructState(const ct::String& name)
{
    return new State(name);
}

ct::String StateMachine::toDot() const
{
    ct::String dot = "digraph StateMachine {\n";

    char number[32];
    for (State* state : mStates)
    {
        dot += "\t";
        dot += state->name();
        dot += "[label = \"";
        dot += state->name();
        dot += "\\nInit: ";
        snprintf(number, sizeof(number), "%0.3f", state->initialValue());
        dot += number;
        dot += "\\nValue: ";
        snprintf(number, sizeof(number), "%0.3f", state->value());
        dot += number;
        dot += "\"]";
        if (state == mCurrentState)
            dot += "[color=red]";
        dot += ";\n";
    }

    for (State* state : mStates)
    {
        for (Transition* trans : state->transitions())
        {
            dot += "\t";
            dot += trans->source().name();
            dot += "->";
            dot += trans->target().name();
            dot += "[label = \"";
            dot += trans->label();
            dot += "\"];\n";
        }
    }
    dot += "};\n";
    return dot;
}

}
