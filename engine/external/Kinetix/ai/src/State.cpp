#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "State.h"

#include "Action.h"
#include "Transition.h"

namespace kinetix::AI
{

State::~State()
{
    for (Transition* t : mTransitions)
        delete t;
    mTransitions.clear();

    for (Action* a : mActions)
        delete a;
    mActions.clear();

    for (Action* a : mEnterActions)
        delete a;
    mEnterActions.clear();

    for (Action* a : mExitActions)
        delete a;
    mExitActions.clear();
}

// By index with size() re-read: an action may add actions to this very list,
// and a reallocation would leave a range-for iterator dangling.
void State::runActions(ct::Vector<Action*>& actions)
{
    for (usize i = 0; i < actions.size(); ++i)
        actions[i]->execute();
}

void State::reset()
{
    mValue = mInitialValue;
    enter();
}

void State::enter()
{
    runActions(mEnterActions);
}

void State::iterate()
{
    runActions(mActions);
}

void State::exit()
{
    runActions(mExitActions);
}

}
