#pragma once

#include "prisma3d/scene/Node.h"

namespace prisma3d
{

class NodeFactory
{
public:
    template<typename T, typename... Args>
    static T* make(Args&&... args)
    {
        return new T(static_cast<Args&&>(args)...);
    }

    static void attach(Node* parent, Node* child)
    {
        parent->addChild(child);
    }

    static void destroy(Node* node)
    {
        delete node;
    }
};

} // namespace prisma3d
