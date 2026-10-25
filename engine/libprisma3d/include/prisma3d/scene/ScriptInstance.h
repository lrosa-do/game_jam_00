#pragma once

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Node;
struct InputEvent;

class ScriptInstance
{
public:
    virtual ~ScriptInstance()
    {
    }

    virtual bool hasScript() const = 0;
    virtual const ct::String& getPath() const = 0;
    virtual void notification(int what) = 0;
    virtual void input(const InputEvent& event) = 0;
    virtual void bodyEntered(Node* other)
    {
        (void) other;
    }
    virtual void bodyExited(Node* other)
    {
        (void) other;
    }
    virtual void buttonPressed(Node* button)
    {
        (void) button;
    }
    virtual void textSubmitted(Node* field, const ct::String& text)
    {
        (void) field;
        (void) text;
    }
    virtual void textChanged(Node* field, const ct::String& text)
    {
        (void) field;
        (void) text;
    }
    virtual bool wantsBodyContacts() const
    {
        return false;
    }
    virtual void getPropertyList(ct::Vector<PropertyInfo>& list) const = 0;
    virtual bool setProperty(const ct::String& name, const PropertyValue& value) = 0;
    virtual bool getProperty(const ct::String& name, PropertyValue& value) const = 0;
    virtual bool getDefaultProperty(const ct::String& name, PropertyValue& value) const = 0;
};

} // namespace prisma3d
