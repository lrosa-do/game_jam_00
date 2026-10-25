#include "prisma3d/scene/ManualMesh3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include "platform.h"

#include <ct/json.hpp>

namespace prisma3d
{

namespace
{

const char* const kMeshName = "manual mesh";

std::uint32_t attributeMaskOf(const SurfaceData& data)
{
    std::uint32_t mask = kVertexPosition | kVertexNormal | kVertexColor;
    if (!data.tangents.empty()) mask |= kVertexTangent;
    if (!data.uvs.empty()) mask |= kVertexUv;
    if (!data.uvs2.empty()) mask |= kVertexUv2;
    return mask;
}

} // namespace

ManualMesh3D::ManualMesh3D()
        : VisualInstance3D(),
          mSurfaces(),
          mGpuMaterials(),
          mMesh(),
          mVertexCapacity(0),
          mIndexCapacity(0),
          mActive(false),
          mActiveTopology(prisma::Topology::Triangles),
          mActiveMaterial(),
          mBuild(),
          mUsesColors(false),
          mUsesNormals(false),
          mUsesTangents(false),
          mUsesUvs(false),
          mUsesUv2s(false),
          mCurrentColor(1.0f, 1.0f, 1.0f, 1.0f),
          mCurrentNormal(0.0f, 0.0f, 0.0f),
          mCurrentTangent(0.0f, 0.0f, 0.0f, 0.0f),
          mCurrentUv(0.0f, 0.0f),
          mCurrentUv2(0.0f, 0.0f)
{
}

ManualMesh3D::~ManualMesh3D()
{
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetBase(getInstance(), MeshId());
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
}

void ManualMesh3D::setDynamic(std::uint32_t vertexCapacity, std::uint32_t indexCapacity)
{
    if (vertexCapacity == mVertexCapacity && indexCapacity == mIndexCapacity) return;

    resetActive();
    mSurfaces.clear();
    mGpuMaterials.clear();
    MeshStore* store = MeshStore::singleton();
    if (store && store->valid(mMesh)) store->clearSurfaces(mMesh);

    mVertexCapacity = vertexCapacity;
    mIndexCapacity = indexCapacity;
}

bool ManualMesh3D::isDynamic() const
{
    return mVertexCapacity > 0;
}

std::uint32_t ManualMesh3D::getVertexCapacity() const
{
    return mVertexCapacity;
}

std::uint32_t ManualMesh3D::getIndexCapacity() const
{
    return mIndexCapacity;
}

bool ManualMesh3D::surfaceBegin(prisma::Topology topology, MaterialId material)
{
    if (mActive) return false;

    mActiveTopology = topology;
    mActiveMaterial = material;
    mActive = true;
    return true;
}

void ManualMesh3D::surfaceSetColor(const Math::Vec4& color)
{
    if (!mActive) return;

    if (!mUsesColors)
    {
        mBuild.colors.resize(mBuild.positions.size(), color);
        mUsesColors = true;
    }
    mCurrentColor = color;
}

void ManualMesh3D::surfaceSetNormal(const Math::Vec3& normal)
{
    if (!mActive) return;

    if (!mUsesNormals)
    {
        mBuild.normals.resize(mBuild.positions.size(), normal);
        mUsesNormals = true;
    }
    mCurrentNormal = normal;
}

void ManualMesh3D::surfaceSetTangent(const Math::Vec4& tangent)
{
    if (!mActive) return;

    if (!mUsesTangents)
    {
        mBuild.tangents.resize(mBuild.positions.size(), tangent);
        mUsesTangents = true;
    }
    mCurrentTangent = tangent;
}

void ManualMesh3D::surfaceSetUv(const Math::Vec2& uv)
{
    if (!mActive) return;

    if (!mUsesUvs)
    {
        mBuild.uvs.resize(mBuild.positions.size(), uv);
        mUsesUvs = true;
    }
    mCurrentUv = uv;
}

void ManualMesh3D::surfaceSetUv2(const Math::Vec2& uv2)
{
    if (!mActive) return;

    if (!mUsesUv2s)
    {
        mBuild.uvs2.resize(mBuild.positions.size(), uv2);
        mUsesUv2s = true;
    }
    mCurrentUv2 = uv2;
}

void ManualMesh3D::surfaceAddVertex(const Math::Vec3& position)
{
    if (!mActive) return;

    if (mUsesColors) mBuild.colors.push_back(mCurrentColor);
    if (mUsesNormals) mBuild.normals.push_back(mCurrentNormal);
    if (mUsesTangents) mBuild.tangents.push_back(mCurrentTangent);
    if (mUsesUvs) mBuild.uvs.push_back(mCurrentUv);
    if (mUsesUv2s) mBuild.uvs2.push_back(mCurrentUv2);
    mBuild.positions.push_back(position);
}

void ManualMesh3D::surfaceAddIndex(std::uint32_t index)
{
    if (!mActive) return;

    mBuild.indices.push_back(index);
}

bool ManualMesh3D::surfaceEnd()
{
    if (!mActive) return false;
    if (mBuild.positions.empty()) return false;

    mBuild.topology = mActiveTopology;
    const int index = commitSurface(mBuild, mActiveMaterial);
    resetActive();
    return index >= 0;
}

bool ManualMesh3D::isSurfaceActive() const
{
    return mActive;
}

int ManualMesh3D::addSurface(const SurfaceData& surface, MaterialId material)
{
    if (mActive) return -1;
    return commitSurface(surface, material);
}

int ManualMesh3D::commitSurface(const SurfaceData& data, MaterialId material)
{
    if (data.positions.empty()) return -1;
    if (!ensureMesh()) return -1;

    const int index = static_cast<int>(mSurfaces.size());
    const bool ok =
            isDynamic() ? uploadDynamic(data, material, index) : uploadStatic(data, material);
    if (!ok) return -1;

    Surface surface;
    surface.data = data;
    surface.material = material;
    mSurfaces.push_back(surface);
    return index;
}

void ManualMesh3D::clearSurfaces()
{
    resetActive();
    mSurfaces.clear();

    MeshStore* store = MeshStore::singleton();
    if (!store || !store->valid(mMesh)) return;

    if (isDynamic())
    {
        for (std::size_t i = 0; i < mGpuMaterials.size(); ++i) blankGpuSurface(static_cast<int>(i));
    }
    else
    {
        store->clearSurfaces(mMesh);
        mGpuMaterials.clear();
    }
}

int ManualMesh3D::getSurfaceCount() const
{
    return static_cast<int>(mSurfaces.size());
}

const SurfaceData* ManualMesh3D::getSurfaceData(int surface) const
{
    if (surface < 0 || static_cast<std::size_t>(surface) >= mSurfaces.size()) return nullptr;
    return &mSurfaces[static_cast<std::size_t>(surface)].data;
}

void ManualMesh3D::setSurfaceMaterial(int surface, MaterialId material)
{
    if (surface < 0 || static_cast<std::size_t>(surface) >= mSurfaces.size()) return;

    mSurfaces[static_cast<std::size_t>(surface)].material = material;
    MeshStore* store = MeshStore::singleton();
    if (store && store->setSurfaceMaterial(mMesh, static_cast<std::uint32_t>(surface), material) &&
            static_cast<std::size_t>(surface) < mGpuMaterials.size())
        mGpuMaterials[static_cast<std::size_t>(surface)] = material;
}

bool ManualMesh3D::setSurfaceMaterialPath(int surface, const ct::String& path)
{
    if (surface < 0 || surface >= getSurfaceCount()) return false;
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        log_error("manual mesh: %s", sceneres::takeError().c_str());
        return false;
    }
    setSurfaceMaterial(surface, material);
    return true;
}

MaterialId ManualMesh3D::getSurfaceMaterial(int surface) const
{
    if (surface < 0 || static_cast<std::size_t>(surface) >= mSurfaces.size()) return MaterialId();
    return mSurfaces[static_cast<std::size_t>(surface)].material;
}

MeshId ManualMesh3D::getMesh() const
{
    return mMesh;
}

bool ManualMesh3D::buildCollision(CollisionMesh& out) const
{
    out.clear();
    for (std::size_t i = 0; i < mSurfaces.size(); ++i)
        if (mSurfaces[i].data.topology == prisma::Topology::Triangles)
            out.append(mSurfaces[i].data);
    return out.finish();
}

void ManualMesh3D::resetActive()
{
    mBuild.positions.clear();
    mBuild.normals.clear();
    mBuild.tangents.clear();
    mBuild.colors.clear();
    mBuild.uvs.clear();
    mBuild.uvs2.clear();
    mBuild.indices.clear();
    mUsesColors = false;
    mUsesNormals = false;
    mUsesTangents = false;
    mUsesUvs = false;
    mUsesUv2s = false;
    mActiveMaterial = MaterialId();
    mActive = false;
}

bool ManualMesh3D::ensureMesh()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return false;
    if (store->valid(mMesh)) return true;

    mMesh = store->create(kMeshName);
    mGpuMaterials.clear();
    setBase(mMesh);
    return mMesh.valid();
}

bool ManualMesh3D::uploadStatic(const SurfaceData& data, MaterialId material)
{
    MeshStore* store = MeshStore::singleton();
    if (!store->addSurface(mMesh, data, kMeshName)) return false;

    const std::uint32_t surface = static_cast<std::uint32_t>(mSurfaces.size());
    if (material.valid()) store->setSurfaceMaterial(mMesh, surface, material);
    return true;
}

bool ManualMesh3D::uploadDynamic(const SurfaceData& data, MaterialId material, int index)
{
    MeshStore* store = MeshStore::singleton();
    const std::size_t slot = static_cast<std::size_t>(index);

    if (slot >= mGpuMaterials.size())
    {
        SurfaceLayout layout;
        layout.topology = data.topology;
        layout.attributeMask = attributeMaskOf(data);
        layout.vertexCount = mVertexCapacity;
        layout.indexCount = mIndexCapacity;
        if (store->addDynamicSurface(mMesh, layout, kMeshName) != index) return false;
        mGpuMaterials.push_back(MaterialId());
    }

    if (!store->updateSurface(mMesh, static_cast<std::uint32_t>(index), data))
    {
        blankGpuSurface(index);
        return false;
    }

    if (!(mGpuMaterials[slot] == material))
    {
        store->setSurfaceMaterial(mMesh, static_cast<std::uint32_t>(index), material);
        mGpuMaterials[slot] = material;
    }
    return true;
}

void ManualMesh3D::blankGpuSurface(int index)
{
    MeshStore* store = MeshStore::singleton();
    if (store) store->updateSurface(mMesh, static_cast<std::uint32_t>(index), SurfaceData());
}

void ManualMesh3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ManualMesh3D;
    DB::addPropertyGroup<P>("Dynamic", "dynamic_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "dynamic_vertex_capacity", 0.0, 16777216.0, 1.0,
                    true),
            [](P* mesh, const PropertyValue& value) {
                mesh->setDynamic(static_cast<std::uint32_t>(value.asInt()),
                        mesh->getIndexCapacity());
            },
            &P::getVertexCapacity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "dynamic_index_capacity", 0.0, 16777216.0, 1.0,
                    true),
            [](P* mesh, const PropertyValue& value) {
                mesh->setDynamic(mesh->getVertexCapacity(),
                        static_cast<std::uint32_t>(value.asInt()));
            },
            &P::getIndexCapacity);
}

namespace
{

constexpr std::int64_t kManualMeshDataVersion = 1;

}

bool ManualMesh3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    if (mSurfaces.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kManualMeshDataVersion));
    ct::Json surfaces = ct::Json::array();
    for (std::size_t i = 0; i < mSurfaces.size(); ++i)
    {
        ct::Json entry = ct::Json::object();
        entry["surface"] = scenejson::fromSurface(mSurfaces[i].data);
        if (mSurfaces[i].material.valid())
        {
            const ct::String ref = sceneres::pathOf(mSurfaces[i].material);
            if (ref.empty())
                result.addWarning(scenejson::element(path, "surfaces", i),
                        "the material has no path; the surface material was not saved");
            else
                entry["material"] = ct::Json(ref);
        }
        surfaces.push_back(entry);
    }
    data["surfaces"] = surfaces;
    return true;
}

bool ManualMesh3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kManualMeshDataVersion, result, path)) return false;

    clearSurfaces();
    const ct::Json* surfaces = scenejson::arrayField(data, "surfaces", result, path);
    if (!surfaces) return true;
    for (std::size_t i = 0; i < surfaces->size(); ++i)
    {
        const ct::Json& entry = surfaces->at(i);
        const ct::String entryPath = scenejson::element(path, "surfaces", i);
        const ct::Json* surfaceField = entry.is_object() ? entry.find("surface") : nullptr;
        if (!surfaceField)
        {
            result.addError(entryPath, "missing surface");
            continue;
        }
        SurfaceData surface;
        if (!scenejson::toSurface(*surfaceField, surface, result,
                    scenejson::field(entryPath, "surface")))
            continue;
        MaterialId material;
        const ct::String ref = scenejson::getString(entry, "material", ct::String());
        if (!ref.empty())
        {
            sceneres::clearError();
            material = sceneres::loadMaterial(ref);
            if (!material.valid())
                result.addWarning(scenejson::field(entryPath, "material"), sceneres::takeError());
        }
        if (addSurface(surface, material) < 0)
            result.addError(entryPath, "the surface could not be added");
    }
    return true;
}

} // namespace prisma3d
