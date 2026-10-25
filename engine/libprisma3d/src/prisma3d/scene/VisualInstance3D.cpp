#include "prisma3d/scene/VisualInstance3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Viewport.h"

#include <ct/json.hpp>

namespace prisma3d
{

VisualInstance3D::VisualInstance3D()
        : Node3D(),
          mInstance(),
          mBase(),
          mLayers(1),
          mVisibleInWorld(false),
          mCastShadows(true),
          mTransparency(0.0f),
          mGiMode(GiMode::Static),
          mLightmapTexelScale(1.0f),
          mReflector(nullptr),
          mReflectorPath()
{
    defineAncestry(NodeClass::VisualInstance3D);

    RenderingServer* server = RenderingServer::singleton();
    if (server) mInstance = server->instanceCreate();

    setNotifyTransform(true);
}

VisualInstance3D::~VisualInstance3D()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceFree(mInstance);
}

InstanceId VisualInstance3D::getInstance() const
{
    return mInstance;
}

void VisualInstance3D::setBase(MeshId base)
{
    mBase = base;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetBase(mInstance, base);
}

void VisualInstance3D::setBase(MultiMeshId base)
{
    mBase = MeshId();

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetBase(mInstance, base);
}

MeshId VisualInstance3D::getBase() const
{
    return mBase;
}

void VisualInstance3D::setLayerMask(std::uint32_t mask)
{
    mLayers = mask;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetLayerMask(mInstance, mask);
}

std::uint32_t VisualInstance3D::getLayerMask() const
{
    return mLayers;
}

void VisualInstance3D::setLayerMaskValue(int layerNumber, bool enable)
{
    if (layerNumber < 1 || layerNumber > 20) return;

    std::uint32_t mask = getLayerMask();
    if (enable) mask |= 1u << (layerNumber - 1);
    else
        mask &= ~(1u << (layerNumber - 1));
    setLayerMask(mask);
}

bool VisualInstance3D::getLayerMaskValue(int layerNumber) const
{
    if (layerNumber < 1 || layerNumber > 20) return false;

    return (mLayers & (1u << (layerNumber - 1))) != 0;
}

void VisualInstance3D::setCastShadows(bool cast)
{
    mCastShadows = cast;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetCastShadows(mInstance, cast);
}

bool VisualInstance3D::getCastShadows() const
{
    return mCastShadows;
}

void VisualInstance3D::setTransparency(float transparency)
{
    mTransparency = transparency < 0.0f ? 0.0f : (transparency > 1.0f ? 1.0f : transparency);

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceGeometrySetTransparency(mInstance, mTransparency);
}

float VisualInstance3D::getTransparency() const
{
    return mTransparency;
}

void VisualInstance3D::setGiMode(GiMode mode)
{
    mGiMode = mode;
}

GiMode VisualInstance3D::getGiMode() const
{
    return mGiMode;
}

void VisualInstance3D::setLightmapTexelScale(float scale)
{
    mLightmapTexelScale = scale > 0.01f ? scale : 0.01f;
}

float VisualInstance3D::getLightmapTexelScale() const
{
    return mLightmapTexelScale;
}

void VisualInstance3D::setReflector(PlanarReflector3D* reflector)
{
    mReflectorPath = ct::String();
    mReflector = reflector;
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetReflector(mInstance,
                reflector ? reflector->getReflector() : ReflectorId());
}

void VisualInstance3D::setReflectorPath(const ct::String& path)
{
    mReflectorPath = path;
    resolveReflector();
    notifyPropertyListChanged();
}

ct::String VisualInstance3D::getReflectorPath() const
{
    if (mReflector) return mReflector->isInternal() ? ct::String() : getPathTo(mReflector);
    return mReflectorPath;
}

void VisualInstance3D::resolveReflector()
{
    PlanarReflector3D* reflector = nullptr;
    if (!mReflectorPath.empty())
    {
        Node* node = getNode(mReflectorPath);
        reflector = node ? node->castTo<PlanarReflector3D>() : nullptr;
    }
    if (reflector)
    {
        setReflector(reflector);
        return;
    }
    const ct::String pending = mReflectorPath;
    setReflector(nullptr);
    mReflectorPath = pending;
}

PlanarReflector3D* VisualInstance3D::getReflector() const
{
    return mReflector;
}

Math::Box VisualInstance3D::getAabb() const
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || !mInstance.valid()) return Math::Box();
    return server->instanceGetAabb(mInstance);
}

void VisualInstance3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    RenderingServer* server = RenderingServer::singleton();

    switch (what)
    {
        case kNotificationEnterTree:
            if (!mReflectorPath.empty()) resolveReflector();
            break;
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->instanceSetScenario(mInstance, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server)
                server->instanceSetTransform(mInstance, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->instanceSetScenario(mInstance, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void VisualInstance3D::updateVisibility()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();

    const bool alreadyVisible = mVisibleInWorld;
    const bool visible = isVisibleInTree();
    mVisibleInWorld = visible;

    if (!server) return;

    if (visible && !alreadyVisible) server->instanceSetTransform(mInstance, getGlobalTransform());

    server->instanceSetVisible(mInstance, visible);
}

MeshInstance3D::MeshInstance3D()
        : VisualInstance3D(),
          mMesh(),
          mColor(1.0f, 1.0f, 1.0f, 1.0f),
          mSurfaceMaterials(),
          mHiddenSurfaces(),
          mMaterialOverride(),
          mSkeletonPath("..")
{
    defineAncestry(NodeClass::MeshInstance3D);
}

MeshInstance3D::~MeshInstance3D()
{
}

namespace
{

bool meshHasSkin(MeshId mesh)
{
    MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = store ? store->get(mesh) : nullptr;
    if (!info) return false;
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
        if (info->surfaces[i].skinned()) return true;
    return false;
}

} // namespace

void MeshInstance3D::setMesh(MeshId mesh)
{
    const bool wasSkinned = meshHasSkin(mMesh);
    mMesh = mesh;
    setBase(mesh);
    if (wasSkinned != meshHasSkin(mesh)) notifyPropertyListChanged();
}

void MeshInstance3D::validateProperty(PropertyInfo& info) const
{
    VisualInstance3D::validateProperty(info);
    if (info.name == "skeleton" && !meshHasSkin(mMesh)) info.usage = PropertyUsage::NoEditor;
}

MeshId MeshInstance3D::getMesh() const
{
    return mMesh;
}

void MeshInstance3D::setColor(const Math::Vec4& color)
{
    mColor = color;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetColor(getInstance(), color);
}

Math::Vec4 MeshInstance3D::getColor() const
{
    return mColor;
}

void MeshInstance3D::setSurfaceMaterial(std::uint32_t surface, MaterialId material)
{
    if (surface >= mSurfaceMaterials.size())
    {
        if (!material.valid()) return;
        mSurfaceMaterials.resize(surface + 1);
    }
    mSurfaceMaterials[surface] = material;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetSurfaceMaterial(getInstance(), surface, material);
}

MaterialId MeshInstance3D::getSurfaceMaterial(std::uint32_t surface) const
{
    return surface < mSurfaceMaterials.size() ? mSurfaceMaterials[surface] : MaterialId();
}

void MeshInstance3D::setSurfaceVisible(std::uint32_t surface, bool visible)
{
    if (surface >= mHiddenSurfaces.size())
    {
        if (visible) return;
        mHiddenSurfaces.resize(surface + 1, 0);
    }
    mHiddenSurfaces[surface] = visible ? 0 : 1;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetSurfaceVisible(getInstance(), surface, visible);
}

bool MeshInstance3D::isSurfaceVisible(std::uint32_t surface) const
{
    return surface >= mHiddenSurfaces.size() || mHiddenSurfaces[surface] == 0;
}

void MeshInstance3D::setMaterialOverride(MaterialId material)
{
    mMaterialOverride = material;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetMaterialOverride(getInstance(), material);
}

MaterialId MeshInstance3D::getMaterialOverride() const
{
    return mMaterialOverride;
}

void MeshInstance3D::setSkeletonPath(const ct::String& path)
{
    mSkeletonPath = path;
    if (isInsideTree()) resolveSkeleton();
}

const ct::String& MeshInstance3D::getSkeletonPath() const
{
    return mSkeletonPath;
}

Skeleton3D* MeshInstance3D::getSkeleton() const
{
    if (mSkeletonPath.empty()) return nullptr;
    Node* node = getNode(mSkeletonPath);
    return node ? node->castTo<Skeleton3D>() : nullptr;
}

void MeshInstance3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            resolveSkeleton();
            break;
        case kNotificationExitTree:
            if (RenderingServer* server = RenderingServer::singleton())
                server->instanceAttachSkeleton(getInstance(), SkeletonId());
            break;
        default:
            break;
    }
}

void MeshInstance3D::resolveSkeleton()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    Skeleton3D* skeleton = getSkeleton();
    server->instanceAttachSkeleton(getInstance(),
            skeleton ? skeleton->getSkeletonId() : SkeletonId());
}

Math::Box MeshInstance3D::getAabb() const
{
    MeshStore* meshes = MeshStore::singleton();
    if (!meshes) return Math::Box();

    const MeshInfo* info = meshes->get(mMesh);
    if (!info) return Math::Box();
    return info->aabb;
}

void VisualInstance3D::refreshReflectorChoices(Node* root)
{
    if (!root) return;
    if (VisualInstance3D* visual = root->castTo<VisualInstance3D>())
        visual->notifyPropertyListChanged();
    for (std::size_t i = 0; i < root->getChildCount(); ++i)
        refreshReflectorChoices(root->getChild(i));
}

void VisualInstance3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name != "reflector" || !mReflectorPath.empty() || mReflector) return;
    const SceneTree* tree = getTree();
    if (!tree || tree->getPlanarReflectorCount() == 0) info.usage = PropertyUsage::NoEditor;
}

void VisualInstance3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<VisualInstance3D>("Layers", "layers", true);
    DB::addProperty<VisualInstance3D>(PropertyInfo::layers3DRender("layers"),
            &VisualInstance3D::setLayerMask, &VisualInstance3D::getLayerMask);

    DB::addPropertyGroup<VisualInstance3D>("Geometry", "");
    DB::addProperty<VisualInstance3D>(PropertyInfo(PropertyType::Bool, "cast_shadow"),
            &VisualInstance3D::setCastShadows, &VisualInstance3D::getCastShadows);
    DB::addProperty<VisualInstance3D>(
            PropertyInfo::range(PropertyType::Float, "transparency", 0.0, 1.0, 0.01),
            &VisualInstance3D::setTransparency, &VisualInstance3D::getTransparency);
    DB::addProperty<VisualInstance3D>(PropertyInfo::nodePath("reflector", "PlanarReflector3D"),
            &VisualInstance3D::setReflectorPath, &VisualInstance3D::getReflectorPath);

    DB::addPropertyGroup<VisualInstance3D>("Global Illumination", "gi_");
    DB::addProperty<VisualInstance3D>(
            PropertyInfo::enumeration("gi_mode", "Disabled,Static,Dynamic"),
            &VisualInstance3D::setGiMode, &VisualInstance3D::getGiMode);
    DB::addProperty<VisualInstance3D>(PropertyInfo::range(PropertyType::Float,
                                              "gi_lightmap_texel_scale", 0.01, 10.0, 0.0001, true),
            &VisualInstance3D::setLightmapTexelScale, &VisualInstance3D::getLightmapTexelScale);
}

void MeshInstance3D::bindProperties()
{
    using DB = NodeClassDB;
    resourceprop::add<MeshInstance3D, MeshId>(PropertyInfo::resource("mesh", "Mesh"),
            &MeshInstance3D::setMesh, &MeshInstance3D::getMesh);
    DB::addProperty<MeshInstance3D>(PropertyInfo::color("color"), &MeshInstance3D::setColor,
            &MeshInstance3D::getColor);
    resourceprop::add<MeshInstance3D, MaterialId>(
            PropertyInfo::resource("material_override", "Material"),
            &MeshInstance3D::setMaterialOverride, &MeshInstance3D::getMaterialOverride);

    DB::addPropertyGroup<MeshInstance3D>("Skeleton", "");
    DB::addProperty<MeshInstance3D>(PropertyInfo::nodePath("skeleton", "Skeleton3D"),
            &MeshInstance3D::setSkeletonPath, &MeshInstance3D::getSkeletonPath);
}

namespace
{

constexpr std::int64_t kMeshInstanceDataVersion = 1;

}

bool MeshInstance3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    ct::Json materials = ct::Json::array();
    for (std::size_t i = 0; i < mSurfaceMaterials.size(); ++i)
    {
        if (!mSurfaceMaterials[i].valid()) continue;
        const ct::String ref = sceneres::pathOf(mSurfaceMaterials[i]);
        if (ref.empty())
        {
            result.addWarning(scenejson::element(path, "surface_materials", i),
                    "the material has no path; the surface material was not saved");
            continue;
        }
        ct::Json entry = ct::Json::object();
        entry["surface"] = ct::Json(static_cast<long long>(i));
        entry["material"] = ct::Json(ref);
        materials.push_back(entry);
    }
    ct::Json hidden = ct::Json::array();
    for (std::size_t i = 0; i < mHiddenSurfaces.size(); ++i)
        if (mHiddenSurfaces[i] != 0) hidden.push_back(ct::Json(static_cast<long long>(i)));
    if (materials.size() == 0 && hidden.size() == 0) return false;

    data["version"] = ct::Json(static_cast<long long>(kMeshInstanceDataVersion));
    if (materials.size() > 0) data["surface_materials"] = materials;
    if (hidden.size() > 0) data["hidden_surfaces"] = hidden;
    return true;
}

bool MeshInstance3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kMeshInstanceDataVersion, result, path)) return false;

    if (const ct::Json* materials = scenejson::arrayField(data, "surface_materials", result, path))
    {
        for (std::size_t i = 0; i < materials->size(); ++i)
        {
            const ct::Json& entry = materials->at(i);
            const ct::String entryPath = scenejson::element(path, "surface_materials", i);
            const std::int64_t surface =
                    entry.is_object() ? scenejson::getInt(entry, "surface", -1) : -1;
            const ct::String ref = entry.is_object()
                                           ? scenejson::getString(entry, "material", ct::String())
                                           : ct::String();
            if (surface < 0 || ref.empty())
            {
                result.addError(entryPath, "expected a surface index and a material");
                continue;
            }
            sceneres::clearError();
            const MaterialId material = sceneres::loadMaterial(ref);
            if (!material.valid())
            {
                result.addWarning(scenejson::field(entryPath, "material"), sceneres::takeError());
                continue;
            }
            setSurfaceMaterial(static_cast<std::uint32_t>(surface), material);
        }
    }
    if (const ct::Json* hidden = scenejson::arrayField(data, "hidden_surfaces", result, path))
    {
        for (std::size_t i = 0; i < hidden->size(); ++i)
        {
            std::int64_t surface = 0;
            if (!scenejson::toInt(hidden->at(i), surface) || surface < 0)
            {
                result.addError(scenejson::element(path, "hidden_surfaces", i),
                        "not a surface index");
                continue;
            }
            setSurfaceVisible(static_cast<std::uint32_t>(surface), false);
        }
    }
    return true;
}

} // namespace prisma3d
