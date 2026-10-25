#include "prisma3d/scene/ReflectionProbe3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Viewport.h"

#include <math.h>

namespace prisma3d
{

ReflectionProbe3D::ReflectionProbe3D()
        : Node3D(),
          mProbe(),
          mParams(),
          mVisibleInWorld(false)
{
    setNotifyTransform(true);
    RenderingServer* server = RenderingServer::singleton();
    if (server)
    {
        mProbe = server->reflectionProbeCreate();
        server->reflectionProbeSetParams(mProbe, mParams);
    }
}

ReflectionProbe3D::~ReflectionProbe3D()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->reflectionProbeFree(mProbe);
}

ReflectionProbeId ReflectionProbe3D::getProbe() const
{
    return mProbe;
}

void ReflectionProbe3D::push()
{
    if (RenderingServer* server = RenderingServer::singleton())
        server->reflectionProbeSetParams(mProbe, mParams);
}

void ReflectionProbe3D::setUpdateMode(ReflectionProbeUpdateMode mode)
{
    mParams.updateMode = mode;
    push();
}

ReflectionProbeUpdateMode ReflectionProbe3D::getUpdateMode() const
{
    return mParams.updateMode;
}

void ReflectionProbe3D::setIntensity(float intensity)
{
    mParams.intensity = fmaxf(intensity, 0.0f);
    push();
}

float ReflectionProbe3D::getIntensity() const
{
    return mParams.intensity;
}

void ReflectionProbe3D::setBlendDistance(float distance)
{
    mParams.blendDistance = fmaxf(distance, 0.0f);
    push();
}

float ReflectionProbe3D::getBlendDistance() const
{
    return mParams.blendDistance;
}

void ReflectionProbe3D::setAmbientMode(ReflectionProbeAmbientMode mode)
{
    const bool changed = mParams.ambientMode != mode;
    mParams.ambientMode = mode;
    push();
    if (changed) notifyPropertyListChanged();
}

void ReflectionProbe3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if ((info.name == "ambient_color" || info.name == "ambient_color_energy") &&
            mParams.ambientMode != ReflectionProbeAmbientMode::Color)
        info.usage = PropertyUsage::NoEditor;
}

ReflectionProbeAmbientMode ReflectionProbe3D::getAmbientMode() const
{
    return mParams.ambientMode;
}

void ReflectionProbe3D::setAmbientColor(const Math::Vec3& color)
{
    mParams.ambientColor = color;
    push();
}

const Math::Vec3& ReflectionProbe3D::getAmbientColor() const
{
    return mParams.ambientColor;
}

void ReflectionProbe3D::setAmbientEnergy(float energy)
{
    mParams.ambientEnergy = fmaxf(energy, 0.0f);
    push();
}

float ReflectionProbe3D::getAmbientEnergy() const
{
    return mParams.ambientEnergy;
}

void ReflectionProbe3D::setMaxDistance(float distance)
{
    mParams.maxDistance = fmaxf(distance, 0.0f);
    push();
}

float ReflectionProbe3D::getMaxDistance() const
{
    return mParams.maxDistance;
}

void ReflectionProbe3D::setSize(const Math::Vec3& size)
{
    mParams.size = Math::Vec3(fmaxf(size.x, 0.001f), fmaxf(size.y, 0.001f), fmaxf(size.z, 0.001f));
    push();
}

const Math::Vec3& ReflectionProbe3D::getSize() const
{
    return mParams.size;
}

void ReflectionProbe3D::setOriginOffset(const Math::Vec3& offset)
{
    mParams.originOffset = offset;
    push();
}

const Math::Vec3& ReflectionProbe3D::getOriginOffset() const
{
    return mParams.originOffset;
}

void ReflectionProbe3D::setInterior(bool interior)
{
    mParams.interior = interior;
    push();
}

bool ReflectionProbe3D::isInterior() const
{
    return mParams.interior;
}

void ReflectionProbe3D::setBoxProjection(bool enabled)
{
    mParams.boxProjection = enabled;
    push();
}

bool ReflectionProbe3D::isBoxProjection() const
{
    return mParams.boxProjection;
}

void ReflectionProbe3D::setShadows(bool enabled)
{
    mParams.shadows = enabled;
    push();
}

bool ReflectionProbe3D::isShadows() const
{
    return mParams.shadows;
}

void ReflectionProbe3D::setCullMask(std::uint32_t mask)
{
    mParams.cullMask = mask;
    push();
}

std::uint32_t ReflectionProbe3D::getCullMask() const
{
    return mParams.cullMask;
}

void ReflectionProbe3D::setReflectionMask(std::uint32_t mask)
{
    mParams.reflectionMask = mask;
    push();
}

std::uint32_t ReflectionProbe3D::getReflectionMask() const
{
    return mParams.reflectionMask;
}

void ReflectionProbe3D::setMeshLodThreshold(float pixels)
{
    mParams.meshLodThreshold = fmaxf(pixels, 0.0f);
    push();
}

float ReflectionProbe3D::getMeshLodThreshold() const
{
    return mParams.meshLodThreshold;
}

const ReflectionProbeParams& ReflectionProbe3D::getParams() const
{
    return mParams;
}

Math::Box ReflectionProbe3D::getAabb() const
{
    const Math::Vec3 half = mParams.size * 0.5f;
    return Math::Box(half * -1.0f, half);
}

void ReflectionProbe3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    RenderingServer* server = RenderingServer::singleton();

    switch (what)
    {
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->reflectionProbeSetScenario(mProbe, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server)
                server->reflectionProbeSetTransform(mProbe, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->reflectionProbeSetScenario(mProbe, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void ReflectionProbe3D::updateVisibility()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();
    mVisibleInWorld = isVisibleInTree();
    if (!server) return;
    server->reflectionProbeSetVisible(mProbe, mVisibleInWorld);
    if (mVisibleInWorld) server->reflectionProbeSetTransform(mProbe, getGlobalTransform());
}

void ReflectionProbe3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ReflectionProbe3D;
    DB::addProperty<P>(PropertyInfo::enumeration("update_mode", "Once (Fast),Always (Slow)"),
            &P::setUpdateMode, &P::getUpdateMode);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "intensity", 0.0, 1.0, 0.01, true),
            &P::setIntensity, &P::getIntensity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "blend_distance", 0.0, 8.0, 0.01, true),
            &P::setBlendDistance, &P::getBlendDistance);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "max_distance", 0.0, 16384.0, 0.1, true),
            &P::setMaxDistance, &P::getMaxDistance);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "size"), &P::setSize, &P::getSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "origin_offset"), &P::setOriginOffset,
            &P::getOriginOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "box_projection"), &P::setBoxProjection,
            &P::isBoxProjection);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "interior"), &P::setInterior,
            &P::isInterior);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "enable_shadows"), &P::setShadows,
            &P::isShadows);
    DB::addProperty<P>(PropertyInfo::layers3DRender("cull_mask"), &P::setCullMask, &P::getCullMask);
    DB::addProperty<P>(PropertyInfo::layers3DRender("reflection_mask"), &P::setReflectionMask,
            &P::getReflectionMask);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "mesh_lod_threshold", 0.0, 1024.0, 0.1),
            &P::setMeshLodThreshold, &P::getMeshLodThreshold);

    DB::addPropertyGroup<P>("Ambient", "ambient_");
    DB::addProperty<P>(
            PropertyInfo::enumeration("ambient_mode", "Disabled,Environment,Constant Color"),
            &P::setAmbientMode, &P::getAmbientMode);
    DB::addProperty<P>(PropertyInfo::color("ambient_color", true), &P::setAmbientColor,
            &P::getAmbientColor);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "ambient_color_energy", 0.0, 16.0, 0.01),
            &P::setAmbientEnergy, &P::getAmbientEnergy);
}

} // namespace prisma3d
