#include "check.h"

#include <Action.h>
#include <State.h>
#include <StateMachine.h>
#include <Transition.h>

using namespace kinetix::AI;

namespace
{

class Always final : public Transition
{
public:
    using Transition::Transition;
    bool shouldTransition() override
    {
        return true;
    }
};

}

int main()
{
    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);
        a->addTransition(new Always(a, b));
        a->addExitAction(new CallbackAction(a, [&machine, b](State&) { machine.removeState(b); }));
        machine.iterate();
        CHECK(machine.currentState() == a);
        CHECK(machine.states().size() == 1);
        CHECK(machine.findState("B") == nullptr);
        machine.iterate();
        CHECK(machine.currentState() == a);
        CHECK(machine.toDot().size() > 0);
    }

    {
        StateMachine machine;
        State* a = new State("A");
        machine.addState(a);
        a->addAction(new CallbackAction(a, [&machine, a](State&) { machine.removeState(a); }));
        a->addAction(new AddAction(a, 1.0f));
        machine.iterate();
        CHECK(machine.currentState() == nullptr);
        CHECK(machine.states().empty());
        machine.iterate();
    }

    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);
        a->addTransition(new CallbackTransition(a, b, [&machine, a](State&)
        {
            machine.removeState(a);
            return true;
        }));
        b->addTransition(new CallbackTransition(b, a, [&machine, b](State&)
        {
            machine.removeState(b);
            return false;
        }));
        machine.iterate();
        CHECK(machine.currentState() == nullptr);
        CHECK(machine.states().size() == 1);
    }

    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);
        b->addTransition(new Always(b, a));
        a->addEnterAction(new CallbackAction(a, [&machine, b](State&) { machine.removeState(b); }));
        machine.reset();
        CHECK(machine.currentState() == a);
        CHECK(machine.states().size() == 1);
    }

    {
        StateMachine machine;
        State* a = new State("A");
        machine.addState(a);
        a->addAction(new CallbackAction(a, [a](State&)
        {
            for (int i = 0; i < 16; ++i)
                a->addAction(new AddAction(a, 1.0f));
        }));
        machine.iterate();
        CHECK(a->value() == 16.0f);
    }

    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);
        a->addTransition(new Always(a, b));
        machine.removeState(b);
        CHECK(a->transitions().empty());
        machine.iterate();
        CHECK(machine.currentState() == a);
    }

    return TEST_RESULT();
}
