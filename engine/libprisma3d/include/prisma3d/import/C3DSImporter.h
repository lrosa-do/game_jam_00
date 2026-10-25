#pragma once

#include "prisma3d/import/ModelImporter.h"

namespace prisma3d
{

class C3DSImporter final : public ModelImporter
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
};

} // namespace prisma3d
