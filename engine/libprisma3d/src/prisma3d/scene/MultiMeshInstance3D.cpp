#include "prisma3d/scene/MultiMeshInstance3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include <ct/json.hpp>

namespace prisma3d
{

MultiMeshInstance3D::MultiMeshInstance3D()
        : VisualInstance3D(),
          mMultiMesh(new MultiMesh()),
          mSurfaceMaterials(),
          mMaterialOverride()
{

    setBase(mMultiMesh->getId());
}

MultiMeshInstance3D::~MultiMeshInstance3D()
{
    delete mMultiMesh;
}

MultiMesh* MultiMeshInstance3D::getMultiMesh() const
{
    return mMultiMesh;
}

void MultiMeshInstance3D::setMesh(MeshId mesh)
{
    mMultiMesh->setMesh(mesh);
}

MeshId MultiMeshInstance3D::getMesh() const
{
    return mMultiMesh->getMesh();
}

bool MultiMeshInstance3D::setUseColors(bool enable)
{
    return mMultiMesh->setUseColors(enable);
}

bool MultiMeshInstance3D::getUseColors() const
{
    return mMultiMesh->getUseColors();
}

bool MultiMeshInstance3D::setUseCustomData(bool enable)
{
    return mMultiMesh->setUseCustomData(enable);
}

bool MultiMeshInstance3D::getUseCustomData() const
{
    return mMultiMesh->getUseCustomData();
}

void MultiMeshInstance3D::setInstanceCount(int count)
{
    mMultiMesh->setInstanceCount(count);
}

int MultiMeshInstance3D::getInstanceCount() const
{
    return mMultiMesh->getInstanceCount();
}

bool MultiMeshInstance3D::setVisibleInstanceCount(int count)
{
    return mMultiMesh->setVisibleInstanceCount(count);
}

int MultiMeshInstance3D::getVisibleInstanceCount() const
{
    return mMultiMesh->getVisibleInstanceCount();
}

void MultiMeshInstance3D::setInstanceTransform(int instance, const Math::Mat4& transform)
{
    mMultiMesh->setInstanceTransform(instance, transform);
}

Math::Mat4 MultiMeshInstance3D::getInstanceTransform(int instance) const
{
    return mMultiMesh->getInstanceTransform(instance);
}

void MultiMeshInstance3D::setInstanceColor(int instance, const Math::Vec4& color)
{
    mMultiMesh->setInstanceColor(instance, color);
}

Math::Vec4 MultiMeshInstance3D::getInstanceColor(int instance) const
{
    return mMultiMesh->getInstanceColor(instance);
}

void MultiMeshInstance3D::setInstanceCustomData(int instance, const Math::Vec4& data)
{
    mMultiMesh->setInstanceCustomData(instance, data);
}

Math::Vec4 MultiMeshInstance3D::getInstanceCustomData(int instance) const
{
    return mMultiMesh->getInstanceCustomData(instance);
}

void MultiMeshInstance3D::setSurfaceMaterial(std::uint32_t surface, MaterialId material)
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

MaterialId MultiMeshInstance3D::getSurfaceMaterial(std::uint32_t surface) const
{
    return surface < mSurfaceMaterials.size() ? mSurfaceMaterials[surface] : MaterialId();
}

void MultiMeshInstance3D::setMaterialOverride(MaterialId material)
{
    mMaterialOverride = material;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetMaterialOverride(getInstance(), material);
}

MaterialId MultiMeshInstance3D::getMaterialOverride() const
{
    return mMaterialOverride;
}

Math::Box MultiMeshInstance3D::getAabb() const
{
    return mMultiMesh->getAabb();
}

void MultiMeshInstance3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = MultiMeshInstance3D;
    resourceprop::add<P, MeshId>(PropertyInfo::resource("mesh", "Mesh"), &P::setMesh, &P::getMesh);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("material_override", "Material"),
            &P::setMaterialOverride, &P::getMaterialOverride);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_colors"), &P::setUseColors,
            &P::getUseColors);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_custom_data"), &P::setUseCustomData,
            &P::getUseCustomData);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "instance_count", 0.0, 16777216.0, 1.0, true),
            &P::setInstanceCount, &P::getInstanceCount);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "visible_instance_count", -1.0,
                               16777216.0, 1.0, true),
            &P::setVisibleInstanceCount, &P::getVisibleInstanceCount);
}

namespace
{

constexpr std::int64_t kMultiMeshDataVersion = 1;

}

bool MultiMeshInstance3D::saveData(ct::Json& data, SceneResult& result,
        const ct::String& path) const
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

    ct::Vector<float> buffer;
    const bool hasBuffer = mMultiMesh->getInstanceCount() > 0 && mMultiMesh->getBuffer(buffer);
    if (!hasBuffer && materials.size() == 0) return false;

    data["version"] = ct::Json(static_cast<long long>(kMultiMeshDataVersion));
    if (materials.size() > 0) data["surface_materials"] = materials;
    if (hasBuffer) data["buffer"] = scenejson::fromFloatBlock(buffer.data(), buffer.size());
    return true;
}

bool MultiMeshInstance3D::loadData(const ct::Json& data, SceneResult& result,
        const ct::String& path)
{
    if (!scenejson::checkVersion(data, kMultiMeshDataVersion, result, path)) return false;

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

    if (const ct::Json* block = data.find("buffer"))
    {
        ct::Vector<float> buffer;
        if (!scenejson::toFloatBlock(*block, buffer))
        {
            result.addError(scenejson::field(path, "buffer"), "not a valid block of numbers");
            return false;
        }
        if (!mMultiMesh->setBuffer(buffer.data(), buffer.size()))
        {
            result.addError(scenejson::field(path, "buffer"),
                    "the instance data does not fit the instance count and the colors and "
                    "custom data flags");
            return false;
        }
    }
    return true;
}

} // namespace prisma3d
