#pragma once

#include "prisma3d/import/ModelImporter.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class AssetLoader;
class MaterialStore;

struct PMeshSaveOptions
{
    ct::Vector<ct::String> skeletonFiles;
    const ct::Vector<ct::String>* materialNames = nullptr;
    bool generateTangents = false;
};

struct PMeshFiles
{
    ct::Vector<std::uint8_t> mesh;
    ct::Vector<ct::String> skeletonNames;
    ct::Vector<ct::Vector<std::uint8_t>> skeletons;
    ct::Vector<ct::String> animationNames;
    ct::Vector<ct::Vector<std::uint8_t>> animations;
};

struct PMeshConvertOptions
{
    bool bakeTangents = true;
};

struct PMeshSurfaceEdit
{
    ct::String mesh;
    ct::Vector<std::uint32_t> remove;
    ct::Vector<std::uint32_t> extract;
};

bool buildPMesh(const ModelData& model, const ModelImporter& importer, const ct::String& fileStem,
        const PMeshSaveOptions& options, PMeshFiles& out, ct::String& error);
bool buildPMeshSkeleton(const Skeleton& skeleton, ct::Vector<std::uint8_t>& out, ct::String& error);
bool buildPMeshAnimation(const AnimationClip& clip, const ct::Vector<Skeleton>& skeletons,
        ct::Vector<std::uint8_t>& out, ct::String& error);
bool savePMesh(const ModelData& model, const ModelImporter& importer, const ct::String& path,
        const PMeshSaveOptions& options, ct::String& error);
bool convertModelToPMesh(AssetLoader& loader, MaterialStore& materials, const ct::String& source,
        const ct::String& destination, const PMeshConvertOptions& options, ct::String& error);
bool editModelToPMesh(AssetLoader& loader, MaterialStore& materials, const ct::String& source,
        const PMeshSurfaceEdit& edit, const ct::String& destination,
        ct::Vector<ct::String>& extracted, ct::String& error);

} // namespace prisma3d
