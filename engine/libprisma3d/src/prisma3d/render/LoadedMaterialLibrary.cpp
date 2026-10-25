#include "prisma3d/render/LoadedMaterialLibrary.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

bool isAbsolute(const ct::String& path)
{
    if (path.empty()) return false;
    if (path[0] == '/') return true;
    return path.size() >= 2 && path[1] == ':';
}

ct::String directoryOf(const ct::String& path)
{
    const ct::String normalized = FileSystem::normalizeSeparators(path);
    std::size_t slash = normalized.size();
    while (slash > 0 && normalized[slash - 1] != '/') --slash;
    if (slash == 0) return ct::String();
    return ct::String(normalized.c_str(), slash - 1);
}

void splitSegments(const ct::String& path, ct::Vector<ct::String>& out)
{
    ct::String current;
    for (std::size_t i = 0; i <= path.size(); ++i)
    {
        if (i == path.size() || path[i] == '/')
        {
            if (!current.empty()) out.push_back(current);
            current = ct::String();
        }
        else
            current += path[i];
    }
}

ct::String normalizePath(const ct::String& path)
{
    return FileSystem::simplifyPath(path);
}

bool isSrgbSlot(MaterialTexture slot)
{
    return slot == MaterialTexture::Albedo || slot == MaterialTexture::Emission ||
           slot == MaterialTexture::DetailAlbedo;
}

ct::String indexedName(std::size_t index)
{
    char text[40];
    snprintf(text, sizeof(text), "material_%u", static_cast<unsigned>(index));
    return ct::String(text);
}

ct::String cleanName(const ct::String& name)
{
    ct::String result;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        result += (c == '"' || c == '\n' || c == '\r') ? '_' : c;
    }
    return result;
}

bool saveTo(const MaterialStore& store, const ct::Vector<MaterialId>& materials,
        const ct::String& diskPath, const ct::String& logicalPath, ct::String& error)
{
    MaterialLibraryData data;
    captureMaterials(store, materials, data);
    for (std::size_t i = 0; i < data.materials.size(); ++i)
    {
        for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
        {
            if (!data.materials[i].paths[slot].empty())
                data.materials[i].paths[slot] =
                        relativeMaterialTexturePath(logicalPath, data.materials[i].paths[slot]);
            ct::Vector<ct::String>& sequence = data.materials[i].sequences[slot];
            for (std::size_t frame = 0; frame < sequence.size(); ++frame)
                sequence[frame] = relativeMaterialTexturePath(logicalPath, sequence[frame]);
        }
    }
    return MaterialLibraryFormat::save(diskPath, data, error);
}

} // namespace

ct::String materialSidecarPath(const ct::String& modelPath)
{
    std::size_t nameStart = 0;
    for (std::size_t i = 0; i < modelPath.size(); ++i)
        if (modelPath[i] == '/' || modelPath[i] == '\\') nameStart = i + 1;
    std::size_t end = modelPath.size();
    for (std::size_t i = modelPath.size(); i > nameStart; --i)
    {
        if (modelPath[i - 1] == '.')
        {
            end = i - 1;
            break;
        }
    }
    ct::String result(modelPath.c_str(), end);
    result += ".mat";
    return result;
}

ct::String resolveMaterialTexturePath(const ct::String& baseDirectory, const ct::String& file)
{
    if (file.empty()) return ct::String();
    if (isAbsolute(file) || baseDirectory.empty()) return normalizePath(file);
    return normalizePath(baseDirectory + "/" + file);
}

ct::String relativeMaterialTexturePath(const ct::String& libraryPath, const ct::String& file)
{
    if (file.empty()) return ct::String();
    const ct::String library = normalizePath(directoryOf(libraryPath));
    const ct::String target = normalizePath(file);
    if (library.empty()) return target;
    if (isAbsolute(library) != isAbsolute(target)) return target;

    ct::Vector<ct::String> from;
    ct::Vector<ct::String> to;
    splitSegments(library, from);
    splitSegments(target, to);
    std::size_t common = 0;
    while (common < from.size() && common + 1 < to.size() && from[common] == to[common]) ++common;
    if (common == 0 && isAbsolute(target)) return target;

    ct::String result;
    for (std::size_t i = common; i < from.size(); ++i) result += "../";
    for (std::size_t i = common; i < to.size(); ++i)
    {
        if (i > common) result += "/";
        result += to[i];
    }
    return result;
}

bool captureMaterial(const MaterialStore& store, MaterialId material, MaterialDef& out)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    out.info = *info;
    out.info.name = cleanName(info->name);
    out.info.path = ct::String();
    out.info.version = 0;
    FileSystem& files = FileSystem::getSingleton();
    const TextureStore* textures = TextureStore::singleton();
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
    {
        out.info.textures[slot] = TextureId();
        out.paths[slot] = ct::String();
        out.sequences[slot].clear();
        out.sequenceFps[slot] = 0.0f;
        const TextureInfo* texture = textures ? textures->get(info->textures[slot]) : nullptr;
        if (!texture) continue;
        if (!texture->sequence.empty())
        {
            bool complete = true;
            for (std::size_t frame = 0; frame < texture->sequence.size(); ++frame)
                complete = complete && files.exists(texture->sequence[frame]);
            if (!complete) continue;
            out.sequences[slot] = texture->sequence;
            out.sequenceFps[slot] = texture->sequenceFps;
            continue;
        }
        if (texture->path.empty() || !files.exists(texture->path)) continue;
        out.paths[slot] = texture->path;
    }
    return true;
}

bool applyMaterial(MaterialStore& store, AssetLoader& loader, MaterialId material,
        const MaterialDef& def, const ct::String& baseDirectory)
{
    if (!store.valid(material)) return false;
    const MaterialInfo& m = def.info;
    store.setName(material, m.name.c_str());

    store.setColor(material, MaterialParam::Albedo, m.albedo);
    store.setFloat(material, MaterialParam::Metallic, m.metallic);
    store.setFloat(material, MaterialParam::Specular, m.specular);
    store.setFloat(material, MaterialParam::Roughness, m.roughness);
    store.setColor(material, MaterialParam::Emission,
            Math::Vec4(m.emission.x, m.emission.y, m.emission.z, 1.0f));
    store.setFloat(material, MaterialParam::EmissionEnergy, m.emissionEnergy);
    store.setFloat(material, MaterialParam::NormalScale, m.normalScale);
    store.setFloat(material, MaterialParam::AoLightAffect, m.aoLightAffect);
    store.setFloat(material, MaterialParam::Clearcoat, m.clearcoat);
    store.setFloat(material, MaterialParam::ClearcoatRoughness, m.clearcoatRoughness);
    store.setFloat(material, MaterialParam::AlphaScissorThreshold, m.alphaScissorThreshold);
    store.setVec3(material, MaterialParam::Uv1Scale, m.uv1Scale);
    store.setVec3(material, MaterialParam::Uv1Offset, m.uv1Offset);
    store.setVec3(material, MaterialParam::Uv2Scale, m.uv2Scale);
    store.setVec3(material, MaterialParam::Uv2Offset, m.uv2Offset);

    store.setTransparency(material, m.transparency);
    store.setAlphaAntialiasing(material, m.alphaAntialiasing, m.alphaAntialiasingEdge);
    store.setCull(material, m.cull);
    store.setEmissionOperator(material, m.emissionOperator);
    store.setShading(material, m.shading);
    store.setBlendMode(material, m.blend);
    store.setDepthTest(material, m.depthTest);
    store.setDetail(material, m.detail);
    store.setDetailBlendMode(material, m.detailBlend);
    store.setDetailUv(material, m.detailUv);
    store.setTextureChannel(material, MaterialTexture::Metallic, m.metallicChannel);
    store.setTextureChannel(material, MaterialTexture::Roughness, m.roughnessChannel);
    store.setTextureChannel(material, MaterialTexture::AmbientOcclusion, m.aoChannel);
    store.setTextureChannel(material, MaterialTexture::Refraction, m.refractionChannel);
    store.setFloat(material, MaterialParam::Refraction, m.refraction);
    store.setWater(material, m.water, m.waterData);

    store.setGrass(material, m.grass, m.grassDrawDistance, m.grassCameraBend, m.grassTintA,
            m.grassTintB);
    store.setGrassWind(material, m.grassWindDirection, m.grassWindAmplitude, m.grassWindSpeed);
    store.setLeafWind(material, m.leafWind, m.leafWindHeight, m.leafWindStrength, m.leafWindSpeed,
            m.leafWindDirection);
    store.setBillboard(material, m.billboard, m.billboardKeepScale);
    store.setReceiveShadows(material, m.receiveShadows);
    store.setBacklight(material, m.backlight);
    store.setTerrainMorph(material, m.terrainMorph);

    FileSystem& files = FileSystem::getSingleton();
    for (std::size_t i = 0; i < kMaterialTextureCount; ++i)
    {
        const MaterialTexture slot = static_cast<MaterialTexture>(i);
        TextureId texture;
        const ct::String& file = def.paths[i];
        const ct::Vector<ct::String>& sequence = def.sequences[i];
        if (!sequence.empty())
        {
            ct::Vector<ct::String> resolved;
            for (std::size_t frame = 0; frame < sequence.size(); ++frame)
            {
                ct::String path = resolveMaterialTexturePath(baseDirectory, sequence[frame]);
                if (!files.exists(path)) path = sequence[frame];
                if (!files.exists(path))
                {
                    log_warn("MaterialLibrary: material '%s': texture '%s' not found",
                            m.name.c_str(), sequence[frame].c_str());
                    resolved.clear();
                    break;
                }
                resolved.push_back(path);
            }
            if (!resolved.empty())
                texture = loader.requestAnimatedTexture(resolved, isSrgbSlot(slot),
                        def.sequenceFps[i], true);
        }
        else if (!file.empty())
        {
            ct::String path = resolveMaterialTexturePath(baseDirectory, file);
            if (!files.exists(path)) path = file;
            if (files.exists(path)) texture = loader.requestTexture(path, isSrgbSlot(slot), true);
            else
                log_warn("MaterialLibrary: material '%s': texture '%s' not found", m.name.c_str(),
                        file.c_str());
        }
        store.setTexture(material, slot, texture);
    }
    return true;
}

std::size_t applyMaterialsByNameThenOrder(MaterialStore& store, AssetLoader& loader,
        const ct::Vector<MaterialId>& materials, const MaterialLibraryData& data,
        const ct::String& libraryPath, ct::Vector<ct::String>* unmatched)
{
    const std::size_t count = materials.size();
    const std::size_t available = data.materials.size();
    ct::Vector<std::size_t> source;
    source.resize(count, available);
    ct::Vector<bool> used;
    used.resize(available, false);

    for (std::size_t i = 0; i < count; ++i)
    {
        const MaterialInfo* info = store.get(materials[i]);
        if (!info || info->name.empty()) continue;
        for (std::size_t j = 0; j < available; ++j)
        {
            if (used[j] || !(data.materials[j].info.name == info->name)) continue;
            source[i] = j;
            used[j] = true;
            break;
        }
    }
    if (count == available)
    {
        for (std::size_t i = 0; i < count; ++i)
        {
            if (source[i] != available || used[i] || !store.valid(materials[i])) continue;
            source[i] = i;
            used[i] = true;
        }
    }

    const ct::String baseDirectory = directoryOf(libraryPath);
    std::size_t applied = 0;
    for (std::size_t i = 0; i < count; ++i)
    {
        if (source[i] == available)
        {
            const MaterialInfo* info = store.get(materials[i]);
            if (unmatched && info && !info->name.empty()) unmatched->push_back(info->name);
            continue;
        }
        if (!applyMaterial(store, loader, materials[i], data.materials[source[i]], baseDirectory))
            continue;
        store.setPath(materials[i], libraryPath);
        ++applied;
    }
    return applied;
}

bool applyModelSidecar(MaterialStore& store, AssetLoader& loader, const ct::String& modelPath,
        const ct::Vector<MaterialId>& materials, ct::Vector<ct::String>* unmatched)
{
    const ct::String sidecar = materialSidecarPath(modelPath);
    if (!FileSystem::getSingleton().exists(sidecar)) return false;

    MaterialLibraryData data;
    ct::String error;
    if (!MaterialLibraryFormat::load(sidecar, data, error))
    {
        log_error("MaterialLibrary: %s", error.c_str());
        return false;
    }
    for (std::size_t i = 0; i < data.warnings.size(); ++i)
        log_warn("MaterialLibrary: %s: %s", sidecar.c_str(), data.warnings[i].c_str());
    applyMaterialsByNameThenOrder(store, loader, materials, data, sidecar, unmatched);
    return true;
}

bool captureMaterials(const MaterialStore& store, const ct::Vector<MaterialId>& materials,
        MaterialLibraryData& out)
{
    out.materials.clear();
    out.warnings.clear();
    for (std::size_t i = 0; i < materials.size(); ++i)
    {
        MaterialDef def;
        if (!captureMaterial(store, materials[i], def)) continue;
        if (def.info.name.empty()) def.info.name = indexedName(i);
        if (out.find(def.info.name.c_str()))
        {
            out.warnings.push_back(
                    ct::String("duplicate material name '") + def.info.name + "' skipped");
            continue;
        }
        out.materials.push_back(def);
    }
    return true;
}

bool saveMaterials(const MaterialStore& store, const ct::Vector<MaterialId>& materials,
        const ct::String& path, ct::String& error)
{
    return saveTo(store, materials, path, path, error);
}

bool saveModelMaterials(const MaterialStore& store, const AssetLoader& loader, ModelId model,
        ct::String& error)
{
    const ModelInfo* info = loader.model(model);
    if (!info || info->status != LoadStatus::Loaded)
    {
        error = "the model is not loaded";
        return false;
    }
    const ct::String resolved = FileSystem::getSingleton().resolve(info->path);
    if (resolved.empty())
    {
        error = ct::String("'") + info->path + "' is not a file on disk";
        return false;
    }
    return saveTo(store, info->materials, materialSidecarPath(resolved),
            materialSidecarPath(info->path), error);
}

LoadedMaterialLibrary::LoadedMaterialLibrary(MaterialStore& materials, AssetLoader& loader)
        : mMaterials(materials),
          mLoader(loader)
{
}

LoadedMaterialLibrary::~LoadedMaterialLibrary()
{
    if (MaterialStore::singleton() != &mMaterials) return;
    for (std::size_t i = 0; i < mOwned.size(); ++i) mMaterials.destroy(mOwned[i]);
}

bool LoadedMaterialLibrary::load(const ct::String& path, ct::String& error)
{
    MaterialLibraryData data;
    if (!MaterialLibraryFormat::load(path, data, error)) return false;

    MaterialLibraryData unique;
    unique.warnings = data.warnings;
    for (std::size_t i = 0; i < data.materials.size(); ++i)
    {
        if (unique.find(data.materials[i].info.name.c_str()))
        {
            unique.warnings.push_back(ct::String("duplicate material name '") +
                                      data.materials[i].info.name + "' ignored, the first is used");
            continue;
        }
        unique.materials.push_back(data.materials[i]);
    }

    const ct::String baseDirectory = directoryOf(path);
    for (std::size_t i = 0; i < unique.materials.size(); ++i)
    {
        const MaterialDef& def = unique.materials[i];
        MaterialId id;
        if (const MaterialId* known = mByName.find(def.info.name))
            if (mMaterials.valid(*known)) id = *known;
        if (!id.valid())
        {
            id = mMaterials.create(def.info.name.c_str());
            mByName[def.info.name] = id;
            mOwned.push_back(id);
        }
        applyMaterial(mMaterials, mLoader, id, def, baseDirectory);
        mMaterials.setPath(id, path);
    }

    for (std::size_t i = 0; i < unique.warnings.size(); ++i)
        log_warn("MaterialLibrary: %s: %s", path.c_str(), unique.warnings[i].c_str());
    mData = static_cast<MaterialLibraryData&&>(unique);
    mPath = path;
    error.clear();
    return true;
}

bool LoadedMaterialLibrary::reload(ct::String& error)
{
    if (mPath.empty())
    {
        error = "no library has been loaded";
        return false;
    }
    const ct::String path = mPath;
    return load(path, error);
}

MaterialId LoadedMaterialLibrary::find(const char* name) const
{
    if (!name) return MaterialId();
    const MaterialId* id = mByName.find(ct::String(name));
    return id && mMaterials.valid(*id) ? *id : MaterialId();
}

std::size_t LoadedMaterialLibrary::count() const
{
    return mData.materials.size();
}

const ct::String& LoadedMaterialLibrary::path() const
{
    return mPath;
}

const MaterialLibraryData& LoadedMaterialLibrary::data() const
{
    return mData;
}

std::uint32_t LoadedMaterialLibrary::bind(MeshStore& meshes, MeshId mesh) const
{
    const MeshInfo* info = meshes.get(mesh);
    if (!info) return 0;
    std::uint32_t bound = 0;
    for (std::size_t s = 0; s < info->surfaces.size(); ++s)
    {
        const MaterialInfo* current = mMaterials.get(info->surfaces[s].material);
        if (!current || current->name.empty()) continue;
        const MaterialId replacement = find(current->name.c_str());
        if (!replacement.valid() || replacement == info->surfaces[s].material) continue;
        if (meshes.setSurfaceMaterial(mesh, static_cast<std::uint32_t>(s), replacement)) ++bound;
    }
    return bound;
}

std::uint32_t LoadedMaterialLibrary::bind(MeshStore& meshes, const AssetLoader& loader,
        ModelId model) const
{
    const ModelInfo* info = loader.model(model);
    if (!info || info->status != LoadStatus::Loaded) return 0;
    std::uint32_t bound = 0;
    for (std::size_t m = 0; m < info->meshes.size(); ++m) bound += bind(meshes, info->meshes[m]);
    return bound;
}

} // namespace prisma3d
