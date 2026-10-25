#include "prisma3d/scene/Decal3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/Viewport.h"

#include <math.h>

namespace prisma3d
{

Decal3D::Decal3D()
        : Node3D(),
          mDecal(),
          mParams(),
          mSortingOffset(0.0f),
          mVisibleInWorld(false)
{
    setNotifyTransform(true);
    RenderingServer* server = RenderingServer::singleton();
    if (server) mDecal = server->decalCreate();
}

Decal3D::~Decal3D()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->decalFree(mDecal);
}

DecalId Decal3D::getDecal() const
{
    return mDecal;
}

void Decal3D::setSize(const Math::Vec3& size)
{
    mParams.size = Math::Vec3(fmaxf(size.x, 0.001f), fmaxf(size.y, 0.001f), fmaxf(size.z, 0.001f));
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetSize(mDecal, mParams.size);
}

const Math::Vec3& Decal3D::getSize() const
{
    return mParams.size;
}

void Decal3D::setTexture(DecalTexture type, TextureId texture)
{
    if (type >= DecalTexture::Max) return;
    mParams.textures[static_cast<std::size_t>(type)] = texture;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetTexture(mDecal, type, texture);
}

TextureId Decal3D::getTexture(DecalTexture type) const
{
    return type < DecalTexture::Max ? mParams.textures[static_cast<std::size_t>(type)]
                                    : TextureId();
}

void Decal3D::setEmissionEnergy(float energy)
{
    mParams.emissionEnergy = energy;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetEmissionEnergy(mDecal, energy);
}

float Decal3D::getEmissionEnergy() const
{
    return mParams.emissionEnergy;
}

void Decal3D::setAlbedoMix(float mix)
{
    mParams.albedoMix = mix;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetAlbedoMix(mDecal, mix);
}

float Decal3D::getAlbedoMix() const
{
    return mParams.albedoMix;
}

void Decal3D::setModulate(const Math::Vec4& modulate)
{
    mParams.modulate = modulate;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetModulate(mDecal, modulate);
}

const Math::Vec4& Decal3D::getModulate() const
{
    return mParams.modulate;
}

void Decal3D::setUpperFade(float fade)
{
    mParams.upperFade = fmaxf(fade, 0.0f);
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetFade(mDecal, mParams.upperFade, mParams.lowerFade);
}

float Decal3D::getUpperFade() const
{
    return mParams.upperFade;
}

void Decal3D::setLowerFade(float fade)
{
    mParams.lowerFade = fmaxf(fade, 0.0f);
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetFade(mDecal, mParams.upperFade, mParams.lowerFade);
}

float Decal3D::getLowerFade() const
{
    return mParams.lowerFade;
}

void Decal3D::setNormalFade(float fade)
{
    mParams.normalFade = fade;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetNormalFade(mDecal, fade);
}

float Decal3D::getNormalFade() const
{
    return mParams.normalFade;
}

void Decal3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (!mParams.distanceFade &&
            (info.name == "distance_fade_begin" || info.name == "distance_fade_length"))
        info.usage = PropertyUsage::NoEditor;
}

void Decal3D::setDistanceFadeEnabled(bool enabled)
{
    const bool changed = mParams.distanceFade != enabled;
    mParams.distanceFade = enabled;
    if (changed) notifyPropertyListChanged();
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetDistanceFade(mDecal, mParams.distanceFade, mParams.distanceFadeBegin,
                mParams.distanceFadeLength);
}

bool Decal3D::isDistanceFadeEnabled() const
{
    return mParams.distanceFade;
}

void Decal3D::setDistanceFadeBegin(float distance)
{
    mParams.distanceFadeBegin = distance;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetDistanceFade(mDecal, mParams.distanceFade, mParams.distanceFadeBegin,
                mParams.distanceFadeLength);
}

float Decal3D::getDistanceFadeBegin() const
{
    return mParams.distanceFadeBegin;
}

void Decal3D::setDistanceFadeLength(float distance)
{
    mParams.distanceFadeLength = distance;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetDistanceFade(mDecal, mParams.distanceFade, mParams.distanceFadeBegin,
                mParams.distanceFadeLength);
}

float Decal3D::getDistanceFadeLength() const
{
    return mParams.distanceFadeLength;
}

void Decal3D::setCullMask(std::uint32_t mask)
{
    mParams.cullMask = mask;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetCullMask(mDecal, mask);
}

std::uint32_t Decal3D::getCullMask() const
{
    return mParams.cullMask;
}

void Decal3D::setSortingOffset(float offset)
{
    mSortingOffset = offset;
    if (RenderingServer* server = RenderingServer::singleton())
        server->decalSetSortingOffset(mDecal, offset);
}

float Decal3D::getSortingOffset() const
{
    return mSortingOffset;
}

Math::Box Decal3D::getAabb() const
{
    Math::Box box;
    box = box.Expanded(mParams.size * -0.5f);
    box = box.Expanded(mParams.size * 0.5f);
    return box;
}

void Decal3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    RenderingServer* server = RenderingServer::singleton();

    switch (what)
    {
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->decalSetScenario(mDecal, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server) server->decalSetTransform(mDecal, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->decalSetScenario(mDecal, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void Decal3D::updateVisibility()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();
    mVisibleInWorld = isVisibleInTree();
    if (!server) return;
    server->decalSetVisible(mDecal, mVisibleInWorld);
    if (mVisibleInWorld) server->decalSetTransform(mDecal, getGlobalTransform());
}

void Decal3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Decal3D;
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Vec3, "size", 0.0, 1024.0, 0.001, true),
            &P::setSize, &P::getSize);

    DB::addPropertyGroup<P>("Textures", "texture_");
    auto addTexture = [](const char* name, DecalTexture slot, bool srgb)
    {
        resourceprop::add<P, TextureId>(
                PropertyInfo::resource(name, "Texture2D"),
                [slot](P* decal, TextureId texture) { decal->setTexture(slot, texture); },
                [slot](const P* decal) { return decal->getTexture(slot); }, srgb);
    };
    addTexture("texture_albedo", DecalTexture::Albedo, true);
    addTexture("texture_normal", DecalTexture::Normal, false);
    addTexture("texture_orm", DecalTexture::Orm, false);
    addTexture("texture_emission", DecalTexture::Emission, true);

    DB::addPropertyGroup<P>("Parameters", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "emission_energy", 0.0, 16.0, 0.01, true),
            &P::setEmissionEnergy, &P::getEmissionEnergy);
    DB::addProperty<P>(PropertyInfo::color("modulate"), &P::setModulate, &P::getModulate);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "albedo_mix", 0.0, 1.0, 0.01),
            &P::setAlbedoMix, &P::getAlbedoMix);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "normal_fade", 0.0, 0.999, 0.001),
            &P::setNormalFade, &P::getNormalFade);

    DB::addPropertyGroup<P>("Vertical Fade", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "upper_fade", 0.0, 16.0, 0.001, true),
            &P::setUpperFade, &P::getUpperFade);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "lower_fade", 0.0, 16.0, 0.001, true),
            &P::setLowerFade, &P::getLowerFade);

    DB::addPropertyGroup<P>("Distance Fade", "distance_fade_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "distance_fade_enabled"),
            &P::setDistanceFadeEnabled, &P::isDistanceFadeEnabled);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "distance_fade_begin", 0.0, 4096.0,
                               0.01, true),
            &P::setDistanceFadeBegin, &P::getDistanceFadeBegin);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "distance_fade_length", 0.0, 4096.0,
                               0.01, true),
            &P::setDistanceFadeLength, &P::getDistanceFadeLength);

    DB::addPropertyGroup<P>("Cull Mask", "");
    DB::addProperty<P>(PropertyInfo::layers3DRender("cull_mask"), &P::setCullMask, &P::getCullMask);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "sorting_offset", -1024.0, 1024.0,
                               0.01, true, true),
            &P::setSortingOffset, &P::getSortingOffset);
}

} // namespace prisma3d
