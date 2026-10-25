#pragma once

#include "prisma3d/render/MaterialStore.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

constexpr std::size_t kMaterialTextureCount = static_cast<std::size_t>(MaterialTexture::Count);

constexpr std::size_t kMaxTextureSequence = 256;

struct MaterialDef
{
    MaterialInfo info;
    ct::String paths[kMaterialTextureCount];
    ct::Vector<ct::String> sequences[kMaterialTextureCount];
    float sequenceFps[kMaterialTextureCount] = {};
};

struct MaterialLibraryData
{
    ct::Vector<MaterialDef> materials;
    ct::Vector<ct::String> warnings;

    const MaterialDef* find(const char* name) const;
};

class MaterialLibraryFormat
{
public:
    static const char* textureKey(MaterialTexture slot);

    static bool parse(const char* text, std::size_t size, MaterialLibraryData& out,
            ct::String& error);
    static void write(const MaterialLibraryData& data, ct::String& out);
    static bool load(const ct::String& path, MaterialLibraryData& out, ct::String& error);
    static bool save(const ct::String& path, const MaterialLibraryData& data, ct::String& error);
};

} // namespace prisma3d
