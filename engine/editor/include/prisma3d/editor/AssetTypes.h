#pragma once

#include <igui/widgets/Retained.hpp>
#include <igui/widgets/RetainedBase.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

enum class AssetKind : int
{
    Folder,
    Scene,
    Prefab,
    Model,
    Texture,
    Material,
    Audio,
    Script,
    Other,
    kCount
};

struct AssetIcon
{
    ig::retained::IconId icon;
    ig::retained::Color tint;
};

class AssetTypes
{
public:
    static constexpr const char* kMaterialExtension = "mat";
    static constexpr const char* kScriptExtension = "py";

    static AssetKind classify(const ct::String& name, bool directory);
    static ct::String extensionOf(const ct::String& name);
    static ct::String filePart(const ct::String& path);
    static ct::String lowered(const ct::String& text);
    static bool isModelExtension(const ct::String& extension);
    static bool isTextureExtension(const ct::String& extension);
    static bool isAudioExtension(const ct::String& extension);

    static const char* label(AssetKind kind);
    static AssetIcon icon(AssetKind kind);
};

} // namespace prisma3d
