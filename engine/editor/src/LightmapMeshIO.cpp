#include "prisma3d/editor/LightmapMeshIO.h"

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"
#include "prisma3d/render/RenderingServer.h"

#include <string.h>

namespace prisma3d
{

namespace
{

class SurfaceListImporter final : public ModelImporter
{
public:
    explicit SurfaceListImporter(const ct::Vector<SurfaceData>& surfaces)
            : mSurfaces(surfaces)
    {
    }

    const char* name() const override
    {
        return "surface list";
    }

    bool supports(const char*) const override
    {
        return false;
    }

    bool import(const ct::String&, ModelData&) override
    {
        return false;
    }

    bool importMemory(const std::uint8_t*, std::size_t, const ct::String&, ModelData&) override
    {
        return false;
    }

    bool readPrimitive(const ModelData&, const ModelPrimitiveData& primitive,
            SurfaceData& out) const override
    {
        if (primitive.source >= mSurfaces.size()) return false;
        out = mSurfaces[primitive.source];
        return true;
    }

    bool readImage(const ModelData&, const ModelImageData&,
            ct::Vector<std::uint8_t>&) const override
    {
        return false;
    }

private:
    const ct::Vector<SurfaceData>& mSurfaces;
};

bool allWhite(const ct::Vector<Math::Vec4>& colors)
{
    for (std::size_t i = 0; i < colors.size(); ++i)
        if (colors[i].x != 1.0f || colors[i].y != 1.0f || colors[i].z != 1.0f ||
                colors[i].w != 1.0f)
            return false;
    return true;
}

template<typename T>
void readAttribute(const ct::Vector<std::uint8_t>& bytes, std::uint32_t stride,
        std::uint32_t offset, std::uint32_t count, ct::Vector<T>& out)
{
    out.resize(count);
    for (std::uint32_t v = 0; v < count; ++v)
        memcpy(&out[v], bytes.data() + static_cast<std::size_t>(v) * stride + offset, sizeof(T));
}

} // namespace

bool LightmapMeshIO::isStaticTriangleMesh(MeshId mesh)
{
    MeshStore* meshes = MeshStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(mesh) : nullptr;
    if (!info || info->surfaces.empty()) return false;
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
        if (info->surfaces[i].skinned() ||
                info->surfaces[i].topology != prisma::Topology::Triangles)
            return false;
    return true;
}

bool LightmapMeshIO::hasUv2(MeshId mesh)
{
    MeshStore* meshes = MeshStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(mesh) : nullptr;
    if (!info || info->surfaces.empty()) return false;
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
        if ((info->surfaces[i].attributeMask & kVertexUv2) == 0) return false;
    return true;
}

std::size_t LightmapMeshIO::vertexCount(MeshId mesh)
{
    MeshStore* meshes = MeshStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(mesh) : nullptr;
    std::size_t count = 0;
    if (info)
        for (std::size_t i = 0; i < info->surfaces.size(); ++i)
            count += info->surfaces[i].vertexCount;
    return count;
}

bool LightmapMeshIO::readSurfaces(MeshId mesh, ct::Vector<SurfaceData>& out, ct::String& error)
{
    out.clear();
    error.clear();
    MeshStore* meshes = MeshStore::singleton();
    RenderingServer* server = RenderingServer::singleton();
    const MeshInfo* info = meshes ? meshes->get(mesh) : nullptr;
    if (!info || info->surfaces.empty() || !server)
    {
        error = "there is no mesh to read";
        return false;
    }
    prisma::Driver* driver = server->driver();
    out.resize(info->surfaces.size());
    for (std::size_t s = 0; s < info->surfaces.size(); ++s)
    {
        const SurfaceInfo& surface = info->surfaces[s];
        if (surface.skinned() || surface.topology != prisma::Topology::Triangles)
        {
            error = "only static triangle meshes can be read";
            out.clear();
            return false;
        }
        const std::uint32_t stride = surface.vertexStride;
        const std::uint32_t mask = surface.attributeMask;
        ct::Vector<std::uint8_t> bytes;
        bytes.resize(static_cast<std::size_t>(surface.vertexCount) * stride);
        if (!driver->readBuffer(surface.vertexBuffer, 0, static_cast<std::uint32_t>(bytes.size()),
                    bytes.data()))
        {
            error = "could not read the vertices back from the GPU";
            out.clear();
            return false;
        }
        SurfaceData& data = out[s];
        data.topology = prisma::Topology::Triangles;
        readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexPosition),
                surface.vertexCount, data.positions);
        if (mask & kVertexNormal)
            readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexNormal),
                    surface.vertexCount, data.normals);
        if (mask & kVertexColor)
        {
            readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexColor),
                    surface.vertexCount, data.colors);
            if (allWhite(data.colors)) data.colors.clear();
        }
        if (mask & kVertexTangent)
            readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexTangent),
                    surface.vertexCount, data.tangents);
        if (mask & kVertexUv)
            readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexUv),
                    surface.vertexCount, data.uvs);
        if (mask & kVertexUv2)
            readAttribute(bytes, stride, MeshStore::attributeOffset(mask, kVertexUv2),
                    surface.vertexCount, data.uvs2);

        if (surface.indexCount > 0)
        {
            const bool wide = surface.indexFormat == prisma::IndexFormat::UInt32;
            ct::Vector<std::uint8_t> indexBytes;
            indexBytes.resize(static_cast<std::size_t>(surface.indexCount) * (wide ? 4 : 2));
            if (!driver->readBuffer(surface.indexBuffer, 0,
                        static_cast<std::uint32_t>(indexBytes.size()), indexBytes.data()))
            {
                error = "could not read the indices back from the GPU";
                out.clear();
                return false;
            }
            data.indices.resize(surface.indexCount);
            for (std::uint32_t i = 0; i < surface.indexCount; ++i)
            {
                if (wide) memcpy(&data.indices[i], indexBytes.data() + i * 4, 4);
                else
                {
                    std::uint16_t small = 0;
                    memcpy(&small, indexBytes.data() + i * 2, 2);
                    data.indices[i] = small;
                }
            }
        }
        else
        {
            data.indices.resize(surface.vertexCount);
            for (std::uint32_t i = 0; i < surface.vertexCount; ++i) data.indices[i] = i;
        }
    }
    return true;
}

bool LightmapMeshIO::saveMesh(MeshId source, const ct::Vector<SurfaceData>& surfaces,
        const ct::String& name, const ct::String& path, ct::String& error)
{
    MeshStore* meshes = MeshStore::singleton();
    MaterialStore* materials = MaterialStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(source) : nullptr;
    if (!info || !materials || info->surfaces.size() != surfaces.size())
    {
        error = "the surfaces do not match the mesh";
        return false;
    }

    ct::Vector<MaterialId> distinct;
    ModelData model;
    ModelMeshData mesh;
    mesh.name = name;
    for (std::size_t s = 0; s < surfaces.size(); ++s)
    {
        const SurfaceData& surface = surfaces[s];
        ModelPrimitiveData primitive;
        primitive.topology = prisma::Topology::Triangles;
        primitive.source = static_cast<std::uint32_t>(s);
        primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
        primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
        primitive.attributeMask = kVertexPosition;
        if (!surface.normals.empty()) primitive.attributeMask |= kVertexNormal;
        if (!surface.tangents.empty()) primitive.attributeMask |= kVertexTangent;
        if (!surface.colors.empty()) primitive.attributeMask |= kVertexColor;
        if (!surface.uvs.empty()) primitive.attributeMask |= kVertexUv;
        if (!surface.uvs2.empty()) primitive.attributeMask |= kVertexUv2;
        const MaterialId material = info->surfaces[s].material;
        if (material.valid() && materials->valid(material))
        {
            std::size_t slot = distinct.size();
            for (std::size_t m = 0; m < distinct.size(); ++m)
                if (distinct[m] == material) slot = m;
            if (slot == distinct.size()) distinct.push_back(material);
            primitive.material = static_cast<int>(slot);
        }
        mesh.primitives.push_back(primitive);
    }
    model.meshes.push_back(mesh);

    ct::Vector<ct::String> names;
    for (std::size_t m = 0; m < distinct.size(); ++m)
    {
        ModelMaterialData entry;
        const MaterialInfo* material = materials->get(distinct[m]);
        ct::String label = material ? material->name : ct::String();
        if (label.empty())
        {
            label = "material_";
            label.append_number(static_cast<unsigned long long>(m));
        }
        entry.name = label;
        names.push_back(label);
        model.materials.push_back(entry);
    }

    SurfaceListImporter importer(surfaces);
    PMeshSaveOptions options;
    options.materialNames = &names;
    if (!savePMesh(model, importer, path, options, error)) return false;
    if (!distinct.empty() && !saveMaterials(*materials, distinct, materialSidecarPath(path), error))
        return false;
    return true;
}

} // namespace prisma3d
