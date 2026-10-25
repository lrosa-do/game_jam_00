#include "prisma3d/editor/NodeNaming.h"

#include "prisma3d/scene/Node.h"

namespace prisma3d
{

ct::String NodeNaming::resolve(const Node* node, const ct::String& requested, ct::String& error)
{
    const ct::String validated = Node::validateNodeName(requested);
    if (validated != requested) error = kInvalidNameMessage;

    ct::String name = validated.trimmed();
    if (name.empty()) name = node->getClassName();
    return name;
}

} // namespace prisma3d
