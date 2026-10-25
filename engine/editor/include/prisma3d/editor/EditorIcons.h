#pragma once

#include <igui/widgets/Retained.hpp>

#include <ct/string.hpp>

#include <cstddef>

namespace prisma3d
{

class EditorIcons
{
public:
    static constexpr int kCell = 32;

    static bool load(const char* directory);
    static bool loadDefault();
    static void unload();
    static bool loaded();
    static unsigned int generation();

    static ig::retained::IconId get(const char* name);
    static ig::retained::IconId get(const ct::String& name);
    static bool has(const char* name);

    static std::size_t count();
    static const ct::String& nameAt(std::size_t index);
    static ig::retained::IconId idAt(std::size_t index);
    static ig::retained::Rect uvOf(const char* name);

    static ct::String defaultDirectory();
    static const ct::String& directory();

    static bool parseAtlas(const char* text, int atlasWidth, int atlasHeight);

private:
    static void ensureLoaded();
};

} // namespace prisma3d
