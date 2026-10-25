#include "prisma3d/editor/EditorTreeEdits.h"

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

namespace
{

void gatherOwners(Node* node, ct::Vector<EditorTreeEdits::Owner>& owners)
{
    if (node->getOwner())
    {
        EditorTreeEdits::Owner entry;
        entry.node = node;
        entry.owner = node->getOwner();
        owners.push_back(entry);
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i) gatherOwners(node->getChild(i), owners);
}

void restoreOwners(const ct::Vector<EditorTreeEdits::Owner>& owners)
{
    for (std::size_t i = 0; i < owners.size(); ++i) owners[i].node->setOwner(owners[i].owner);
}

} // namespace

ct::Vector<EditorTreeEdits::Owner> EditorTreeEdits::collectOwners(Node* node)
{
    ct::Vector<Owner> owners;
    if (node) gatherOwners(node, owners);
    return owners;
}

EditorTreeEdits::Placement EditorTreeEdits::capture(Node* node, bool withTransform)
{
    Placement placement;
    placement.node = node;
    placement.parent = node->getParent();
    placement.index = node->getIndex();
    placement.name = node->getName();
    placement.owners = collectOwners(node);
    if (withTransform)
    {
        if (Node3D* spatial = node->castTo<Node3D>())
        {
            placement.hasTransform = true;
            placement.position = spatial->getPosition();
            placement.rotation = spatial->getRotation();
            placement.scale = spatial->getScale();
        }
    }
    return placement;
}

ct::Vector<EditorTreeEdits::Placement> EditorTreeEdits::capture(const ct::Vector<Node*>& nodes,
        bool withTransform)
{
    ct::Vector<Placement> placements;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        placements.push_back(capture(nodes[i], withTransform));
    return placements;
}

void EditorTreeEdits::detach(const ct::Vector<Placement>& placements)
{
    for (std::size_t i = placements.size(); i > 0; --i)
    {
        Node* node = placements[i - 1].node;
        if (Node* parent = node->getParent()) parent->removeChild(node);
    }
}

void EditorTreeEdits::place(const ct::Vector<Placement>& placements)
{
    detach(placements);

    for (std::size_t i = 0; i < placements.size(); ++i)
    {
        const Placement& placement = placements[i];
        if (!placement.parent) continue;
        if (placement.node->getName() != placement.name) placement.node->setName(placement.name);
        placement.parent->addChild(placement.node, true);
        if (placement.index >= 0)
            placement.parent->moveChild(placement.node, static_cast<std::size_t>(placement.index));
    }

    for (std::size_t i = 0; i < placements.size(); ++i)
    {
        const Placement& placement = placements[i];
        if (placement.hasTransform)
        {
            if (Node3D* spatial = placement.node->castTo<Node3D>())
            {
                spatial->setPosition(placement.position);
                spatial->setRotation(placement.rotation);
                spatial->setScale(placement.scale);
            }
        }
        restoreOwners(placement.owners);
    }
}

void EditorTreeEdits::selectNodes(EditorSelection* selection, const ct::Vector<Node*>& nodes)
{
    if (!selection) return;
    selection->clear();
    for (std::size_t i = 0; i < nodes.size(); ++i) selection->add(nodes[i]);
}

ct::Vector<Node*> EditorTreeEdits::childOrder(const Node* parent)
{
    ct::Vector<Node*> order;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i) order.push_back(parent->getChild(i));
    return order;
}

void EditorTreeEdits::restoreOrder(Node* parent, const ct::Vector<Node*>& order)
{
    for (std::size_t i = 0; i < order.size(); ++i)
        if (order[i]->getParent() == parent) parent->moveChild(order[i], i);
}

void EditorTreeEdits::setProperty(EditorUndoRedoManager* undo, const ct::String& action, Node* node,
        const ct::String& property, const PropertyValue& before, const PropertyValue& after)
{
    if (!undo)
    {
        NodeClassDB::setProperty(node, property, after);
        return;
    }
    undo->createPropertyAction(action, node);
    undo->addDoProperty(node, property, after);
    undo->addUndoProperty(node, property, before);
    undo->commitAction();
}

ct::Vector<ct::String> EditorTreeEdits::persistentGroups(const Node* node)
{
    ct::Vector<Node::GroupInfo> groups;
    node->getGroups(groups);
    ct::Vector<ct::String> names;
    for (std::size_t i = 0; i < groups.size(); ++i)
        if (groups[i].persistent) names.push_back(groups[i].name);
    return names;
}

void EditorTreeEdits::setPersistentGroups(Node* node, const ct::Vector<ct::String>& groups)
{
    const ct::Vector<ct::String> current = persistentGroups(node);
    bool same = current.size() == groups.size();
    for (std::size_t i = 0; i < current.size() && same; ++i) same = current[i] == groups[i];
    if (same) return;

    for (std::size_t i = 0; i < current.size(); ++i) node->removeFromGroup(current[i]);
    for (std::size_t i = 0; i < groups.size(); ++i) node->addToGroup(groups[i], true);
}

EditorTreeEdits::Record EditorTreeEdits::added(const ct::String& name,
        const ct::Vector<Node*>& nodes, const ct::Vector<Node*>& selectionBefore)
{
    Record result;
    result.name = name;
    result.after = capture(nodes, true);
    result.before = result.after;
    for (std::size_t i = 0; i < result.before.size(); ++i) result.before[i].parent = nullptr;
    result.selectionBefore = selectionBefore;
    result.selectionAfter = nodes;
    result.doReferences = nodes;
    result.execute = false;
    return result;
}

EditorTreeEdits::Record EditorTreeEdits::removed(const ct::String& name,
        const ct::Vector<Node*>& nodes)
{
    Record result;
    result.name = name;
    result.before = capture(nodes, true);
    result.after = result.before;
    for (std::size_t i = 0; i < result.after.size(); ++i) result.after[i].parent = nullptr;
    result.selectionBefore = nodes;
    result.undoReferences = nodes;
    result.execute = true;
    return result;
}

void EditorTreeEdits::record(EditorUndoRedoManager& undo, EditorSelection* selection,
        const Record& record)
{
    const ct::Vector<Placement> before = record.before;
    const ct::Vector<Placement> after = record.after;
    const ct::Vector<Node*> selectionBefore = record.selectionBefore;
    const ct::Vector<Node*> selectionAfter = record.selectionAfter;
    Node* const scene = record.scene;
    const ct::Vector<Node*> editable = record.editable;

    undo.createAction(record.name);

    if (!editable.empty())
        undo.addDoMethod(
                [scene, editable]
                {
                    for (std::size_t i = 0; i < editable.size(); ++i)
                        scene->setEditableInstance(editable[i], true);
                });
    undo.addDoMethod([after] { place(after); });
    for (std::size_t i = 0; i < record.properties.size(); ++i)
        undo.addDoProperty(record.properties[i].node, record.properties[i].property,
                record.properties[i].after);
    if (record.afterDo) undo.addDoMethod(record.afterDo);
    undo.addDoMethod([selection, selectionAfter] { selectNodes(selection, selectionAfter); });

    undo.addUndoMethod([before] { place(before); });
    for (std::size_t i = 0; i < record.properties.size(); ++i)
        undo.addUndoProperty(record.properties[i].node, record.properties[i].property,
                record.properties[i].before);
    if (!editable.empty())
        undo.addUndoMethod(
                [scene, editable]
                {
                    for (std::size_t i = 0; i < editable.size(); ++i)
                        scene->setEditableInstance(editable[i], false);
                });
    if (record.afterUndo) undo.addUndoMethod(record.afterUndo);
    undo.addUndoMethod([selection, selectionBefore] { selectNodes(selection, selectionBefore); });

    for (std::size_t i = 0; i < record.doReferences.size(); ++i)
        undo.addDoReference(record.doReferences[i]);
    for (std::size_t i = 0; i < record.undoReferences.size(); ++i)
        undo.addUndoReference(record.undoReferences[i]);

    undo.commitAction(record.execute);
}

} // namespace prisma3d
