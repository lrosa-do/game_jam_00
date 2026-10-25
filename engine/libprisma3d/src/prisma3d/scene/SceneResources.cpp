#include "prisma3d/scene/SceneResources.h"

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace sceneres
{

namespace
{

ct::String sError;

void fail(const char* kind, const ct::String& path, const ct::String& reason)
{
    ct::String message = kind;
    message += " '";
    message += path;
    message += "' could not be loaded: ";
    message += reason;
    log_warn("%s", message.c_str());
    sError = message;
}

void splitPart(const ct::String& path, ct::String& file, ct::String& part)
{
    const std::size_t at = path.find("::");
    if (at == ct::String::npos)
    {
        file = path;
        part = ct::String();
        return;
    }
    file = path.substr(0, at);
    part = path.substr(at + 2);
}

ct::String joinPart(const ct::String& file, const ct::String& part)
{
    if (part.empty()) return file;
    ct::String path = file;
    path += "::";
    path += part;
    return path;
}

bool endsWithNoCase(const ct::String& text, const char* suffix)
{
    const std::size_t length = strlen(suffix);
    if (text.size() < length) return false;
    for (std::size_t i = 0; i < length; ++i)
    {
        char a = text[text.size() - length + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = path.size();
    while (slash > 0 && path[slash - 1] != '/') --slash;
    if (slash == 0) return ct::String();
    return ct::String(path.c_str(), slash - 1);
}

const ModelInfo* loadModel(AssetLoader& loader, const ct::String& file, ct::String& error)
{
    MeshStore* meshes = MeshStore::singleton();
    MaterialStore* materials = MaterialStore::singleton();

    const ModelId known = loader.findModelByPath(file);
    if (known.valid())
    {
        if (loader.modelStatus(known) == LoadStatus::InProgress) loader.waitAll();
        const ModelInfo* info = loader.model(known);
        bool usable = info && info->status == LoadStatus::Loaded;
        for (std::size_t i = 0; usable && meshes && i < info->meshes.size(); ++i)
            usable = meshes->valid(info->meshes[i]);
        for (std::size_t i = 0; usable && materials && i < info->materials.size(); ++i)
            usable = materials->valid(info->materials[i]);
        if (usable) return info;
        loader.freeModel(known);
    }

    if (!FileSystem::getSingleton().exists(AssetLoader::modelFile(file)))
    {
        error = "the file was not found";
        return nullptr;
    }
    const ModelId id = loader.requestModel(file);
    if (!id.valid())
    {
        error = "the model could not be requested";
        return nullptr;
    }
    loader.waitAll();
    const ModelInfo* info = loader.model(id);
    if (!info || info->status != LoadStatus::Loaded)
    {
        error = info && !info->error.empty() ? info->error : ct::String("the model did not load");
        loader.freeModel(id);
        return nullptr;
    }
    return info;
}

} // namespace

ct::String normalizePath(const ct::String& path)
{
    if (path.starts_with("primitive:")) return path;
    ct::String file;
    ct::String part;
    splitPart(path, file, part);
    return joinPart(FileSystem::simplifyPath(file), part);
}

bool alive(MeshId mesh)
{
    MeshStore* store = MeshStore::singleton();
    return store && store->valid(mesh);
}

bool alive(TextureId texture)
{
    TextureStore* store = TextureStore::singleton();
    return store && store->valid(texture);
}

bool alive(MaterialId material)
{
    MaterialStore* store = MaterialStore::singleton();
    return store && store->valid(material);
}

bool alive(SoundId sound)
{
    AudioServer* audio = AudioServer::singleton();
    return audio && audio->validSound(sound);
}

ct::String pathOf(MeshId mesh)
{
    MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = store ? store->get(mesh) : nullptr;
    return info ? info->path : ct::String();
}

ct::String pathOf(TextureId texture)
{
    TextureStore* store = TextureStore::singleton();
    const TextureInfo* info = store ? store->get(texture) : nullptr;
    return info ? info->path : ct::String();
}

ct::String pathOf(MaterialId material)
{
    MaterialStore* store = MaterialStore::singleton();
    const MaterialInfo* info = store ? store->get(material) : nullptr;
    if (!info || info->path.empty() || info->name.empty()) return ct::String();
    return joinPart(info->path, info->name);
}

ct::String pathOf(SoundId sound)
{
    AudioServer* audio = AudioServer::singleton();
    const SoundInfo* info = audio ? audio->getSoundInfo(sound) : nullptr;
    return info ? info->path : ct::String();
}

ModelId loadModel(const ct::String& path)
{
    if (path.empty()) return ModelId();
    AssetLoader* loader = AssetLoader::singleton();
    if (!loader)
    {
        fail("model", path, "there is no asset loader");
        return ModelId();
    }
    const ct::String file = normalizePath(path);
    ct::String error;
    if (!loadModel(*loader, file, error))
    {
        fail("model", path, error);
        return ModelId();
    }
    return loader->findModelByPath(file);
}

MeshId loadMesh(const ct::String& path)
{
    if (path.empty()) return MeshId();
    MeshStore* meshes = MeshStore::singleton();
    if (!meshes)
    {
        fail("mesh", path, "there is no mesh store");
        return MeshId();
    }

    ct::String error;
    if (isPrimitivePath(path))
    {
        const MeshId mesh = loadPrimitiveMesh(*meshes, path, error);
        if (!mesh.valid()) fail("mesh", path, error);
        return mesh;
    }

    ct::String file;
    ct::String part;
    splitPart(normalizePath(path), file, part);
    const MeshId cached = meshes->find(joinPart(file, part));
    if (cached.valid()) return cached;

    AssetLoader* loader = AssetLoader::singleton();
    if (!loader)
    {
        fail("mesh", path, "there is no asset loader");
        return MeshId();
    }
    const ModelInfo* model = loadModel(*loader, file, error);
    if (!model)
    {
        fail("mesh", path, error);
        return MeshId();
    }
    if (model->meshes.empty())
    {
        fail("mesh", path, "the model has no meshes");
        return MeshId();
    }
    if (part.empty()) return model->meshes[0];
    const ct::String wanted = joinPart(file, part);
    for (std::size_t i = 0; i < model->meshes.size(); ++i)
    {
        const MeshInfo* info = meshes->get(model->meshes[i]);
        if (info && info->path == wanted) return model->meshes[i];
        const MeshId shadow = meshes->getShadowMesh(model->meshes[i]);
        const MeshInfo* shadowInfo = meshes->get(shadow);
        if (shadowInfo && shadowInfo->path == wanted) return shadow;
    }
    fail("mesh", path, "the model has no such mesh");
    return MeshId();
}

bool reloadModel(const ct::String& path)
{
    MeshStore* meshes = MeshStore::singleton();
    AssetLoader* loader = AssetLoader::singleton();
    if (!meshes || !loader) return false;
    ct::String file;
    ct::String part;
    splitPart(normalizePath(path), file, part);

    ct::Vector<MeshId> previous;
    ct::Vector<ct::String> paths;
    const ModelId known = loader->findModelByPath(file);
    if (known.valid())
    {
        if (loader->modelStatus(known) == LoadStatus::InProgress) loader->waitAll();
        if (const ModelInfo* info = loader->model(known))
            for (std::size_t i = 0; i < info->meshes.size(); ++i)
            {
                const MeshInfo* mesh = meshes->get(info->meshes[i]);
                if (!mesh) continue;
                previous.push_back(info->meshes[i]);
                paths.push_back(mesh->path);
                meshes->setPath(info->meshes[i], ct::String());
            }
        loader->freeModel(known);
    }

    ct::String error;
    const ModelInfo* fresh = loadModel(*loader, file, error);
    if (!fresh)
    {
        fail("mesh", file, error);
        for (std::size_t i = 0; i < previous.size(); ++i) meshes->setPath(previous[i], paths[i]);
        return false;
    }
    const ModelId freshId = loader->findModelByPath(file);
    for (std::size_t i = 0; i < previous.size(); ++i)
    {
        const MeshId replacement = meshes->find(paths[i]);
        const bool adopted = replacement.valid() && meshes->adopt(previous[i], replacement);
        meshes->setPath(previous[i], paths[i]);
        if (!adopted) continue;
        loader->replaceMesh(freshId, replacement, previous[i]);
    }
    return true;
}

bool listModelMeshes(const ct::String& file, ct::Vector<ct::String>& paths)
{
    MeshStore* meshes = MeshStore::singleton();
    AssetLoader* loader = AssetLoader::singleton();
    if (!meshes || !loader)
    {
        fail("model", file, "there is no asset loader");
        return false;
    }
    ct::String error;
    const ModelInfo* model = loadModel(*loader, normalizePath(file), error);
    if (!model)
    {
        fail("model", file, error);
        return false;
    }
    for (std::size_t i = 0; i < model->meshes.size(); ++i)
    {
        const MeshInfo* info = meshes->get(model->meshes[i]);
        if (info) paths.push_back(info->path);
    }
    return true;
}

TextureId loadTexture(const ct::String& path, bool srgb)
{
    if (path.empty()) return TextureId();
    TextureStore* textures = TextureStore::singleton();
    if (!textures)
    {
        fail("texture", path, "there is no texture store");
        return TextureId();
    }

    ct::String file;
    ct::String part;
    splitPart(normalizePath(path), file, part);
    const ct::String canonical = joinPart(file, part);
    AssetLoader* loader = AssetLoader::singleton();

    TextureId id = textures->find(canonical, srgb);
    if (!id.valid() && !part.empty())
    {
        ct::String error;
        if (!loader)
        {
            fail("texture", path, "there is no asset loader");
            return TextureId();
        }
        if (!loadModel(*loader, file, error))
        {
            fail("texture", path, error);
            return TextureId();
        }
        id = textures->find(canonical, srgb);
        if (!id.valid()) id = textures->find(canonical);
        if (!id.valid()) fail("texture", path, "the model has no such image");
        return id;
    }

    if (!id.valid())
    {
        if (!FileSystem::getSingleton().exists(file))
        {
            fail("texture", path, "the file was not found");
            return TextureId();
        }
        if (loader)
        {
            id = loader->requestTexture(file, srgb, true);
            loader->waitAll();
        }
        else
            id = textures->createFromFile(file, srgb, true);
    }
    else if (loader)
    {
        const TextureInfo* info = textures->get(id);
        if (info && info->state == TextureState::Pending) loader->waitAll();
    }

    const TextureInfo* info = textures->get(id);
    if (!info || info->state == TextureState::Failed)
    {
        fail("texture", path, "the image is not valid");
        return TextureId();
    }
    return id;
}

MaterialId loadMaterial(const ct::String& path)
{
    if (path.empty()) return MaterialId();
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials)
    {
        fail("material", path, "there is no material store");
        return MaterialId();
    }

    ct::String file;
    ct::String part;
    splitPart(normalizePath(path), file, part);
    if (part.empty())
    {
        fail("material", path, "a material path needs a '::name' part");
        return MaterialId();
    }
    const MaterialId cached = materials->find(file, part);
    if (cached.valid()) return cached;

    AssetLoader* loader = AssetLoader::singleton();
    if (!loader)
    {
        fail("material", path, "there is no asset loader");
        return MaterialId();
    }

    ct::String error;
    if (endsWithNoCase(file, ".mat"))
    {
        MaterialLibraryData data;
        if (!MaterialLibraryFormat::load(file, data, error))
        {
            fail("material", path, error);
            return MaterialId();
        }
        const MaterialDef* def = data.find(part.c_str());
        if (!def)
        {
            fail("material", path, "the library has no such material");
            return MaterialId();
        }
        const MaterialId id = materials->create(def->info.name.c_str());
        if (!id.valid() || !applyMaterial(*materials, *loader, id, *def, directoryOf(file)))
        {
            fail("material", path, "the material could not be created");
            return MaterialId();
        }
        materials->setPath(id, file);
        loader->waitAll();
        return id;
    }

    const ModelInfo* model = loadModel(*loader, file, error);
    if (!model)
    {
        fail("material", path, error);
        return MaterialId();
    }
    for (std::size_t i = 0; i < model->materials.size(); ++i)
    {
        const MaterialInfo* info = materials->get(model->materials[i]);
        if (info && info->name == part) return model->materials[i];
    }
    fail("material", path, "the model has no such material");
    return MaterialId();
}

SoundId loadSound(const ct::String& path)
{
    if (path.empty()) return SoundId();
    AudioServer* audio = AudioServer::singleton();
    if (!audio || !audio->isReady())
    {
        fail("sound", path, "the audio server is not running");
        return SoundId();
    }
    const ct::String file = normalizePath(path);
    const SoundId known = audio->findSoundByPath(file);
    if (known.valid()) return known;
    if (!FileSystem::getSingleton().exists(file))
    {
        fail("sound", path, "the file was not found");
        return SoundId();
    }
    return audio->requestSoundFromFile(file);
}

ct::String takeError()
{
    ct::String message = sError;
    sError = ct::String();
    return message;
}

void clearError()
{
    sError = ct::String();
}

} // namespace sceneres

} // namespace prisma3d
