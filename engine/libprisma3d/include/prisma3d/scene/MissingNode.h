#pragma once

#include "prisma3d/scene/Node.h"

#include <ct/json.hpp>
#include <ct/string.hpp>

namespace prisma3d
{

class MissingNode : public Node
{
public:
    PRISMA3D_NODE_CLASS(MissingNode, Node);
    static void bindProperties();

    MissingNode();
    ~MissingNode() override;

    void setOriginalClass(const ct::String& className);
    const ct::String& getOriginalClass() const;
    void setOriginalScene(const ct::String& scene);
    const ct::String& getOriginalScene() const;

    void setEntry(const ct::Json& entry);
    const ct::Json& getEntry() const;

private:
    ct::String mOriginalClass;
    ct::String mOriginalScene;
    ct::Json mEntry;
};

} // namespace prisma3d
