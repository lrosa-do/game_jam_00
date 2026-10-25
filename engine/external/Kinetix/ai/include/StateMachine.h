#ifndef KINETIX_AI_STATEMACHINE_H
#define KINETIX_AI_STATEMACHINE_H

// Every action and transition is a user callback that may reach back into the machine.
// removeState() is safe from inside one: the state leaves the machine at once (no longer current,
// findable or a transition target) but is deleted, with the transitions pruned for it, only when
// the outermost iterate()/reset() returns.

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace kinetix::AI
{

class State;
class Transition;

class StateMachine
{
public:
    using StateList = ct::Vector<State*>;

    StateMachine() = default;
    virtual ~StateMachine();

    // Ownership: the StateMachine owns every state added via addState() and
    // deletes them on destruction or when removed (deferred while a callback
    // is running, see above).
    void addState(State* state);
    void removeState(State* state);
    State* findState(const ct::String& name) const;
    State* currentState() const
    {
        return mCurrentState;
    }

    void iterate();
    void reset(); // current = first state (or keeps current), then reset it

    StateList& states()
    {
        return mStates;
    }
    const StateList& states() const
    {
        return mStates;
    }

    // Graphviz .dot dump of states and transitions.
    ct::String toDot() const;

protected:
    // Factory hook so derived machines can build State subclasses; returns a new State owned by the
    // machine.
    virtual State* constructState(const ct::String& name);

    StateList mStates;
    State* mCurrentState = nullptr;

private:
    void step();
    bool contains(const State* state) const;
    void releaseRemoved();

    unsigned mCallbackDepth = 0; // nesting of iterate()/reset() on the stack
    StateList mRemovedStates;
    ct::Vector<Transition*> mRemovedTransitions;
};

}

#endif
