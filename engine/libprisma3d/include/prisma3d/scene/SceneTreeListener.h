#pragma once

namespace prisma3d
{

class Node;

class SceneTreeListener
{
public:
    virtual ~SceneTreeListener()
    {
    }
    virtual void treeChanged()
    {
    }
    virtual void nodeAdded(Node* node)
    {
        (void) node;
    }
    virtual void nodeRemoved(Node* node)
    {
        (void) node;
    }
    virtual void nodeRenamed(Node* node)
    {
        (void) node;
    }
};

} // namespace prisma3d
