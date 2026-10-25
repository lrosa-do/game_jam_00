#pragma once

#include "mathc.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class EditorSelection;
class EditorUndoRedoManager;
class Node;

class EditorTreeEdits
{
public:
    struct Owner
    {
        Node* node = nullptr;
        Node* owner = nullptr;
    };

    struct Placement
    {
        Node* node = nullptr;
        Node* parent = nullptr;
        std::ptrdiff_t index = -1;
        ct::String name;
        bool hasTransform = false;
        Math::Vec3 position;
        Math::Vec3 rotation;
        Math::Vec3 scale;
        ct::Vector<Owner> owners;
    };

    static ct::Vector<Owner> collectOwners(Node* node);
    static Placement capture(Node* node, bool withTransform);
    static ct::Vector<Placement> capture(const ct::Vector<Node*>& nodes, bool withTransform);

    static void detach(const ct::Vector<Placement>& placements);
    static void place(const ct::Vector<Placement>& placements);
    static void selectNodes(EditorSelection* selection, const ct::Vector<Node*>& nodes);
    static void restoreOrder(Node* parent, const ct::Vector<Node*>& order);
    static ct::Vector<Node*> childOrder(const Node* parent);

    struct PropertyChange
    {
        Node* node = nullptr;
        ct::String property;
        PropertyValue before;
        PropertyValue after;
    };

    struct Record
    {
        ct::String name;
        ct::Vector<Placement> before;
        ct::Vector<Placement> after;
        ct::Vector<Node*> selectionBefore;
        ct::Vector<Node*> selectionAfter;
        ct::Vector<Node*> doReferences;
        ct::Vector<Node*> undoReferences;
        ct::Vector<PropertyChange> properties;
        ct::Vector<Node*> editable;
        Node* scene = nullptr;
        bool execute = true;
        ct::Function<void()> afterDo;
        ct::Function<void()> afterUndo;
    };

    static void setProperty(EditorUndoRedoManager* undo, const ct::String& action, Node* node,
            const ct::String& property, const PropertyValue& before, const PropertyValue& after);
    static void setPersistentGroups(Node* node, const ct::Vector<ct::String>& groups);
    static ct::Vector<ct::String> persistentGroups(const Node* node);

    static void record(EditorUndoRedoManager& undo, EditorSelection* selection,
            const Record& record);
    static Record added(const ct::String& name, const ct::Vector<Node*>& nodes,
            const ct::Vector<Node*>& selectionBefore);
    static Record removed(const ct::String& name, const ct::Vector<Node*>& nodes);
};

} // namespace prisma3d
