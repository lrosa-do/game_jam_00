#include "prisma3d/scene/MissingNode.h"

#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

MissingNode::MissingNode()
        : Node(),
          mOriginalClass(),
          mOriginalScene(),
          mEntry(ct::Json::object())
{
}

MissingNode::~MissingNode() = default;

void MissingNode::setOriginalClass(const ct::String& className)
{
    mOriginalClass = className;
}

const ct::String& MissingNode::getOriginalClass() const
{
    return mOriginalClass;
}

void MissingNode::setOriginalScene(const ct::String& scene)
{
    mOriginalScene = scene;
}

const ct::String& MissingNode::getOriginalScene() const
{
    return mOriginalScene;
}

void MissingNode::setEntry(const ct::Json& entry)
{
    mEntry = entry;
}

const ct::Json& MissingNode::getEntry() const
{
    return mEntry;
}

void MissingNode::bindProperties()
{
    using DB = NodeClassDB;
    using P = MissingNode;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "original_class", PropertyHint::None, "",
                               PropertyUsage::None),
            &P::setOriginalClass, &P::getOriginalClass);
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "original_scene", PropertyHint::None, "",
                               PropertyUsage::None),
            &P::setOriginalScene, &P::getOriginalScene);
}

} // namespace prisma3d
