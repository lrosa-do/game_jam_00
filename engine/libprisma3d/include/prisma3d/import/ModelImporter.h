#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Animation.h"

#include "mathc.h"

#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

enum class ModelAlphaMode : std::uint8_t
{
    Opaque,
    Mask,
    Blend
};

struct ModelMaterialData
{
    ct::String name;
    Math::Vec4 baseColor = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    float metallic = 1.0f;
    float roughness = 1.0f;
    Math::Vec3 emissive = Math::Vec3(0.0f, 0.0f, 0.0f);
    float alphaCutoff = 0.5f;
    ModelAlphaMode alphaMode = ModelAlphaMode::Opaque;
    bool doubleSided = false;
    int baseColorTexture = -1;
    int metallicRoughnessTexture = -1;
    int normalTexture = -1;
    int occlusionTexture = -1;
    int emissiveTexture = -1;
};

struct ModelImageData
{
    ct::String name;
    ct::String path;
    ct::String mimeType;
    std::uint64_t size = 0;
    std::uint32_t source = 0;

    bool embedded() const
    {
        return size > 0;
    }
};

struct ModelTextureData
{
    int image = -1;
    bool repeatU = true;
    bool repeatV = true;
    bool nearest = false;
};

struct ModelPrimitiveData
{
    prisma::Topology topology = prisma::Topology::Triangles;
    int material = -1;
    std::uint32_t attributeMask = kVertexPosition;
    std::uint32_t vertexCount = 0;
    std::uint32_t indexCount = 0;
    std::uint64_t sourceBytes = 0;
    std::uint32_t source = 0;
    Math::Box aabb;
    bool hasAabb = false;
    bool skinned = false;
};

struct ModelMeshData
{
    ct::String name;
    ct::Vector<ModelPrimitiveData> primitives;
    ct::Vector<SurfaceData> shadowSurfaces;
    int lightmapImage = -1;
};

struct ModelNodeData
{
    ct::String name;
    int parent = -1;
    ct::Vector<int> children;
    Math::Vec3 translation = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion rotation = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    int mesh = -1;
    int camera = -1;
    int skeleton = -1;
};

struct ModelCameraData
{
    bool perspective = true;
    float yfov = 0.9f;
    float aspect = 0.0f;
    float zNear = 0.1f;
    float zFar = 1000.0f;
    float xmag = 1.0f;
    float ymag = 1.0f;
};

struct ModelSource
{
    virtual ~ModelSource()
    {
    }
};

struct ModelData
{
    ct::String path;
    ct::String directory;
    ct::String error;
    ct::Vector<ModelNodeData> nodes;
    ct::Vector<int> roots;
    ct::Vector<ModelMeshData> meshes;
    ct::Vector<ModelMaterialData> materials;
    ct::Vector<ModelTextureData> textures;
    ct::Vector<ModelImageData> images;
    ct::Vector<ModelCameraData> cameras;
    ct::Vector<Skeleton> skeletons;
    ct::Vector<AnimationClip> animations;
    std::uint64_t geometryBytes = 0;
    std::uint64_t imageBytes = 0;
    std::uint32_t primitiveCount = 0;
    ct::Unique<ModelSource> source;

    Math::Mat4 nodeLocalTransform(int node) const;
    Math::Mat4 nodeGlobalTransform(int node) const;
};

class ModelImporter
{
public:
    virtual ~ModelImporter()
    {
    }

    virtual const char* name() const = 0;
    virtual bool supports(const char* extension) const = 0;
    virtual bool import(const ct::String& path, ModelData& out) = 0;
    virtual bool importMemory(const std::uint8_t* data, std::size_t size,
            const ct::String& virtualName, ModelData& out) = 0;
    virtual bool readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
            SurfaceData& out) const = 0;
    virtual bool readImage(const ModelData& model, const ModelImageData& image,
            ct::Vector<std::uint8_t>& out) const = 0;
};

class ModelLoader
{
public:
    ModelLoader();
    ~ModelLoader();
    ModelLoader(const ModelLoader&) = delete;
    ModelLoader& operator=(const ModelLoader&) = delete;

    static ModelLoader* singleton();

    void addImporter(ModelImporter* importer);
    std::uint32_t importerCount() const;
    ModelImporter* importer(std::uint32_t index) const;
    ModelImporter* importerFor(const ct::String& path) const;

    bool load(const ct::String& path, ModelData& out) const;
    bool loadFromMemory(const std::uint8_t* data, std::size_t size, const ct::String& virtualName,
            ModelData& out) const;

    static ct::String extensionOf(const ct::String& path);

private:
    static ModelLoader* sSingleton;

    ct::Vector<ModelImporter*> mImporters;
};

} // namespace prisma3d
