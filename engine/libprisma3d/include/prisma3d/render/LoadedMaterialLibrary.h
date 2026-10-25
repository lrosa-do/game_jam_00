#pragma once

#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialLibrary.h"
#include "prisma3d/render/MeshStore.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

ct::String materialSidecarPath(const ct::String& modelPath);
ct::String resolveMaterialTexturePath(const ct::String& baseDirectory, const ct::String& file);
ct::String relativeMaterialTexturePath(const ct::String& libraryPath, const ct::String& file);

bool captureMaterial(const MaterialStore& store, MaterialId material, MaterialDef& out);
bool applyMaterial(MaterialStore& store, AssetLoader& loader, MaterialId material,
        const MaterialDef& def, const ct::String& baseDirectory);

std::size_t applyMaterialsByNameThenOrder(MaterialStore& store, AssetLoader& loader,
        const ct::Vector<MaterialId>& materials, const MaterialLibraryData& data,
        const ct::String& libraryPath, ct::Vector<ct::String>* unmatched = nullptr);
bool applyModelSidecar(MaterialStore& store, AssetLoader& loader, const ct::String& modelPath,
        const ct::Vector<MaterialId>& materials, ct::Vector<ct::String>* unmatched = nullptr);

bool captureMaterials(const MaterialStore& store, const ct::Vector<MaterialId>& materials,
        MaterialLibraryData& out);
bool saveMaterials(const MaterialStore& store, const ct::Vector<MaterialId>& materials,
        const ct::String& path, ct::String& error);
bool saveModelMaterials(const MaterialStore& store, const AssetLoader& loader, ModelId model,
        ct::String& error);

class LoadedMaterialLibrary
{
public:
    LoadedMaterialLibrary(MaterialStore& materials, AssetLoader& loader);
    ~LoadedMaterialLibrary();
    LoadedMaterialLibrary(const LoadedMaterialLibrary&) = delete;
    LoadedMaterialLibrary& operator=(const LoadedMaterialLibrary&) = delete;

    bool load(const ct::String& path, ct::String& error);
    bool reload(ct::String& error);

    MaterialId find(const char* name) const;
    std::size_t count() const;
    const ct::String& path() const;
    const MaterialLibraryData& data() const;

    std::uint32_t bind(MeshStore& meshes, MeshId mesh) const;
    std::uint32_t bind(MeshStore& meshes, const AssetLoader& loader, ModelId model) const;

private:
    MaterialStore& mMaterials;
    AssetLoader& mLoader;
    ct::String mPath;
    MaterialLibraryData mData;
    ct::HashMap<ct::String, MaterialId> mByName;
    ct::Vector<MaterialId> mOwned;
};

} // namespace prisma3d
