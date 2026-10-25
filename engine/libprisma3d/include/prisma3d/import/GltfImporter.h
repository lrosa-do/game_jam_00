#pragma once

#include "prisma3d/import/ModelImporter.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

struct IoStream;

namespace prisma3d
{

enum class GltfComponentType : std::uint8_t
{
    Invalid,
    Int8,
    UInt8,
    Int16,
    UInt16,
    UInt32,
    Float
};

struct GltfAccessorRef
{
    int buffer = -1;
    std::uint64_t offset = 0;
    std::uint32_t stride = 0;
    std::uint32_t count = 0;
    std::uint8_t components = 0;
    GltfComponentType componentType = GltfComponentType::Invalid;
    bool normalized = false;

    bool valid() const
    {
        return count > 0 && components > 0 && componentType != GltfComponentType::Invalid;
    }
    std::uint64_t byteLength() const;
};

struct GltfPrimitiveSource
{
    GltfAccessorRef positions;
    GltfAccessorRef normals;
    GltfAccessorRef tangents;
    GltfAccessorRef colors;
    GltfAccessorRef uvs;
    GltfAccessorRef uvs2;
    GltfAccessorRef joints;
    GltfAccessorRef weights;
    GltfAccessorRef indices;

    std::uint64_t byteLength() const;
};

struct GltfImageSource
{
    int buffer = -1;
    std::uint64_t offset = 0;
};

struct GltfSource : ModelSource
{
    bool binary = false;
    std::uint64_t binOffset = 0;
    std::uint64_t binSize = 0;
    ct::Vector<ct::String> bufferPaths;
    ct::Vector<std::uint64_t> bufferSizes;
    ct::Vector<GltfPrimitiveSource> primitives;
    ct::Vector<GltfImageSource> images;
};

class GltfRangeReader
{
public:
    explicit GltfRangeReader(const GltfSource& source);
    ~GltfRangeReader();
    GltfRangeReader(const GltfRangeReader&) = delete;
    GltfRangeReader& operator=(const GltfRangeReader&) = delete;

    bool read(int buffer, std::uint64_t offset, std::uint64_t size, void* out);
    bool read(const GltfAccessorRef& accessor, ct::Vector<std::uint8_t>& out);
    std::uint64_t bytesRead() const;

private:
    IoStream* open(int buffer);

    const GltfSource& mSource;
    ct::Vector<IoStream*> mStreams;
    std::uint64_t mBytesRead;
};

class GltfImporter final : public ModelImporter
{
public:
    const char* name() const override;
    bool supports(const char* extension) const override;
    bool import(const ct::String& path, ModelData& out) override;
    bool importMemory(const std::uint8_t* data, std::size_t size, const ct::String& virtualName,
            ModelData& out) override;
    bool readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
            SurfaceData& out) const override;
    bool readImage(const ModelData& model, const ModelImageData& image,
            ct::Vector<std::uint8_t>& out) const override;

    static const GltfSource* source(const ModelData& model);

    static std::uint32_t componentSize(GltfComponentType type);
    static bool unpackFloats(const GltfAccessorRef& accessor, const std::uint8_t* bytes,
            std::size_t size, float* out, std::uint32_t outComponents);
    static bool unpackIndices(const GltfAccessorRef& accessor, const std::uint8_t* bytes,
            std::size_t size, ct::Vector<std::uint32_t>& out);
};

} // namespace prisma3d
