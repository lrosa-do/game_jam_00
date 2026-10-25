#pragma once

#include "prisma3d/import/ModelImporter.h"

namespace prisma3d
{

class PMeshImporter final : public ModelImporter
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

bool readPMeshSkeleton(const std::uint8_t* data, std::size_t size, Skeleton& out,
        ct::String& error);
bool readPMeshAnimations(const std::uint8_t* data, std::size_t size,
        const ct::Vector<Skeleton>& skeletons, ct::Vector<AnimationClip>& out, ct::String& error);
bool readPMeshLinks(const std::uint8_t* data, std::size_t size, ct::Vector<ct::String>& skeletons,
        ct::Vector<ct::String>& animations, ct::String& error);

} // namespace prisma3d
