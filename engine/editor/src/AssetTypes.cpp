#include "prisma3d/editor/AssetTypes.h"

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/scene/Prefab.h"

#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kModelExtensions[] = { "glb", "gltf", "fbx", "obj", "b3d", "3ds", "ms3d", "mesh",
    "pmesh" };
const char* const kTextureExtensions[] = { "png", "jpg", "jpeg", "tga", "bmp", "dds", "hdr" };
const char* const kAudioExtensions[] = { "wav", "mp3", "flac", "ogg" };

template<std::size_t N>
bool listed(const char* const (&list)[N], const ct::String& extension)
{
    for (std::size_t i = 0; i < N; ++i)
        if (extension == list[i]) return true;
    return false;
}

const char* const kLabels[static_cast<int>(AssetKind::kCount)] = { "Folder", "Scene", "Prefab",
    "Model", "Texture", "Material", "Audio", "Script", "File" };

struct KindIcon
{
    const char* name;
    rt::Color tint;
};

const KindIcon kIcons[static_cast<int>(AssetKind::kCount)] = {
    { "Folder", rt::Color(255, 214, 102, 255) },
    { "PackedScene", rt::Color(255, 255, 255, 255) },
    { "Node3D", rt::Color(255, 255, 255, 255) },
    { "MeshInstance3D", rt::Color(255, 255, 255, 255) },
    { "Texture2D", rt::Color(255, 255, 255, 255) },
    { "StandardMaterial3D", rt::Color(255, 255, 255, 255) },
    { "AudioStreamWAV", rt::Color(255, 255, 255, 255) },
    { "Script", rt::Color(255, 255, 255, 255) },
    { "File", rt::Color(255, 255, 255, 255) },
};

bool endsWithLowered(const ct::String& lowerName, const char* suffix)
{
    return lowerName.ends_with(suffix);
}

} // namespace

ct::String AssetTypes::lowered(const ct::String& text)
{
    ct::String result = text;
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        const char c = result[i];
        if (c >= 'A' && c <= 'Z') result.data()[i] = static_cast<char>(c - 'A' + 'a');
    }
    return result;
}

ct::String AssetTypes::extensionOf(const ct::String& name)
{
    const std::size_t dot = name.rfind('.');
    if (dot == ct::String::npos || dot + 1 >= name.size()) return ct::String();
    return lowered(name.substr(dot + 1));
}

ct::String AssetTypes::filePart(const ct::String& path)
{
    const std::size_t at = path.find("::");
    return at == ct::String::npos ? path : path.substr(0, at);
}

bool AssetTypes::isModelExtension(const ct::String& extension)
{
    return listed(kModelExtensions, extension);
}

bool AssetTypes::isTextureExtension(const ct::String& extension)
{
    return listed(kTextureExtensions, extension);
}

bool AssetTypes::isAudioExtension(const ct::String& extension)
{
    return listed(kAudioExtensions, extension);
}

AssetKind AssetTypes::classify(const ct::String& name, bool directory)
{
    if (directory) return AssetKind::Folder;
    const ct::String lower = lowered(name);
    if (endsWithLowered(lower, EditorDocument::kSceneSuffix)) return AssetKind::Scene;
    if (endsWithLowered(lower, Prefab::kExtension)) return AssetKind::Prefab;
    const ct::String extension = extensionOf(lower);
    if (isModelExtension(extension)) return AssetKind::Model;
    if (isTextureExtension(extension)) return AssetKind::Texture;
    if (extension == kMaterialExtension) return AssetKind::Material;
    if (isAudioExtension(extension)) return AssetKind::Audio;
    if (extension == kScriptExtension) return AssetKind::Script;
    return AssetKind::Other;
}

const char* AssetTypes::label(AssetKind kind)
{
    const int index = static_cast<int>(kind);
    return index >= 0 && index < static_cast<int>(AssetKind::kCount) ? kLabels[index] : "";
}

AssetIcon AssetTypes::icon(AssetKind kind)
{
    const int index = static_cast<int>(kind);
    const KindIcon& found = index >= 0 && index < static_cast<int>(AssetKind::kCount)
                                    ? kIcons[index]
                                    : kIcons[static_cast<int>(AssetKind::Other)];
    return AssetIcon{ EditorIcons::get(found.name), found.tint };
}

} // namespace prisma3d
