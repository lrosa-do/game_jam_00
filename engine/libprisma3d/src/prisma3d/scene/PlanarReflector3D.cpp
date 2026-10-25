#include "prisma3d/scene/PlanarReflector3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

PlanarReflector3D::PlanarReflector3D()
        : Node3D(),
          mReflector(),
          mHalfSize(1.0f, 1.0f),
          mResolution(512),
          mMaxDistance(5.0f),
          mVisibleInWorld(false)
{
    setNotifyTransform(true);
    RenderingServer* server = RenderingServer::singleton();
    if (server) mReflector = server->reflectorCreate(ScenarioId());
}

PlanarReflector3D::~PlanarReflector3D()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->reflectorFree(mReflector);
}

ReflectorId PlanarReflector3D::getReflector() const
{
    return mReflector;
}

void PlanarReflector3D::setHalfSize(const Math::Vec2& halfSize)
{
    mHalfSize = halfSize;
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->reflectorSetHalfSize(mReflector, halfSize);
}

Math::Vec2 PlanarReflector3D::getHalfSize() const
{
    return mHalfSize;
}

void PlanarReflector3D::setResolution(std::uint32_t resolution)
{
    mResolution = resolution;
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->reflectorSetResolution(mReflector, resolution);
}

std::uint32_t PlanarReflector3D::getResolution() const
{
    return mResolution;
}

void PlanarReflector3D::setMaxDistance(float distance)
{
    mMaxDistance = distance;
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->reflectorSetMaxDistance(mReflector, distance);
}

float PlanarReflector3D::getMaxDistance() const
{
    return mMaxDistance;
}

TextureId PlanarReflector3D::getTexture() const
{
    RenderingServer* server = RenderingServer::singleton();
    return server ? server->reflectorGetTexture(mReflector) : TextureId();
}

void PlanarReflector3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    RenderingServer* server = RenderingServer::singleton();
    switch (what)
    {
        case kNotificationEnterTree:
            if (SceneTree* tree = getTree())
            {
                ++tree->mPlanarReflectors;
                VisualInstance3D::refreshReflectorChoices(tree->getRoot());
            }
            break;
        case kNotificationExitTree:
            if (SceneTree* tree = getTree())
            {
                if (tree->mPlanarReflectors > 0) --tree->mPlanarReflectors;
                VisualInstance3D::refreshReflectorChoices(tree->getRoot());
            }
            break;
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->reflectorSetScenario(mReflector, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server)
                server->reflectorSetTransform(mReflector, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->reflectorSetScenario(mReflector, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void PlanarReflector3D::updateVisibility()
{
    if (!isInsideTree()) return;
    RenderingServer* server = RenderingServer::singleton();
    const bool alreadyVisible = mVisibleInWorld;
    const bool visible = isVisibleInTree();
    mVisibleInWorld = visible;
    if (!server) return;
    if (visible && !alreadyVisible) server->reflectorSetTransform(mReflector, getGlobalTransform());
    server->reflectorSetVisible(mReflector, visible);
}

void PlanarReflector3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = PlanarReflector3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "half_size"), &P::setHalfSize,
            &P::getHalfSize);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "resolution", 16.0, 8192.0, 1.0),
            &P::setResolution, &P::getResolution);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "max_distance", 0.0, 4096.0, 0.01, true),
            &P::setMaxDistance, &P::getMaxDistance);
}

} // namespace prisma3d
