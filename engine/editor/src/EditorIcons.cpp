#include "prisma3d/editor/EditorIcons.h"

#include "prisma3d/core/Pixmap.h"

#include "platform.h"

#include <igui/widgets/WidgetApp.hpp>

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

struct Entry
{
    ct::String name;
    rt::IconId icon = rt::IconId::None;
    rt::Rect uv;
};

struct State
{
    ct::Vector<Entry> entries;
    ct::HashMap<ct::String, std::size_t> index;
    rt::TextureHandle texture;
    ct::String directory;
    bool loaded = false;
    bool failed = false;
    unsigned int generation = 0;
};

State& state()
{
    static State instance;
    return instance;
}

ct::String joined(const ct::String& directory, const char* name)
{
    ct::String path = directory;
    if (!path.empty() && path.back() != '/' && path.back() != '\\') path += "/";
    path += name;
    return path;
}

bool isDirectory(const ct::String& path)
{
    PathInfo info;
    return !path.empty() && fs_get_path_info(path.c_str(), &info) &&
           info.type == PATH_TYPE_DIRECTORY;
}

bool nextToken(const char*& cursor, char* out, std::size_t capacity)
{
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r') ++cursor;
    if (*cursor == '\0' || *cursor == '\n') return false;
    std::size_t length = 0;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\r' &&
            *cursor != '\n')
    {
        if (length + 1 >= capacity) return false;
        out[length++] = *cursor++;
    }
    out[length] = '\0';
    return length > 0;
}

} // namespace

bool EditorIcons::parseAtlas(const char* text, int atlasWidth, int atlasHeight)
{
    State& current = state();
    current.entries.clear();
    current.index.clear();
    if (!text || atlasWidth <= 0 || atlasHeight <= 0) return false;

    const float width = static_cast<float>(atlasWidth);
    const float height = static_cast<float>(atlasHeight);
    const char* cursor = text;
    while (*cursor)
    {
        char name[96];
        char numbers[4][24];
        bool good = nextToken(cursor, name, sizeof name);
        for (int i = 0; i < 4 && good; ++i) good = nextToken(cursor, numbers[i], sizeof numbers[i]);
        while (*cursor && *cursor != '\n') ++cursor;
        if (*cursor == '\n') ++cursor;
        if (!good) continue;

        const float x = static_cast<float>(atoi(numbers[0]));
        const float y = static_cast<float>(atoi(numbers[1]));
        const float w = static_cast<float>(atoi(numbers[2]));
        const float h = static_cast<float>(atoi(numbers[3]));
        if (w <= 0.0f || h <= 0.0f || x < 0.0f || y < 0.0f || x + w > width || y + h > height)
            continue;

        Entry entry;
        entry.name = name;
        entry.uv = rt::Rect{ x / width, y / height, w / width, h / height };
        current.index.put(entry.name, current.entries.size());
        current.entries.push_back(entry);
    }
    return !current.entries.empty();
}

bool EditorIcons::load(const char* directory)
{
    State& current = state();
    current.failed = true;
    if (!directory || !*directory) return false;
    const ct::String folder(directory);

    Pixmap pixmap;
    if (!pixmap.load(joined(folder, "atlas.png").c_str())) return false;
    if (pixmap.width <= 0 || pixmap.height <= 0) return false;
    Pixmap* rgba = nullptr;
    if (pixmap.components != 4)
    {
        rgba = pixmap.convert_to_rgba();
        if (!rgba) return false;
    }
    Pixmap& source = rgba ? *rgba : pixmap;
    source.fix_alpha_edges();

    char* listing = file_read_text(joined(folder, "atlas.txt").c_str());
    if (!listing)
    {
        delete rgba;
        return false;
    }

    rt::WidgetApp* app = rt::WidgetApp::existing();
    rt::TextureHandle texture;
    if (app) texture = app->uploadTexture(source.pixels, source.width, source.height);
    const bool parsed = texture && parseAtlas(listing, source.width, source.height);
    free(listing);
    delete rgba;
    if (!parsed)
    {
        if (app) app->destroyTexture(texture);
        current.loaded = false;
        return false;
    }

    if (app) app->destroyTexture(current.texture);
    current.texture = texture;
    for (std::size_t i = 0; i < current.entries.size(); ++i)
        current.entries[i].icon = rt::registerImageIcon(texture, current.entries[i].uv);
    current.directory = folder;
    current.loaded = true;
    current.failed = false;
    ++current.generation;
    return true;
}

void EditorIcons::unload()
{
    State& current = state();
    if (!current.loaded) return;
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->destroyTexture(current.texture);
    current.loaded = false;
    current.failed = false;
    current.entries.clear();
    current.index.clear();
    current.texture = rt::TextureHandle();
    ++current.generation;
}

void EditorIcons::ensureLoaded()
{
    State& current = state();
    if (current.loaded || current.failed || !rt::WidgetApp::existing()) return;
    loadDefault();
}

ct::String EditorIcons::defaultDirectory()
{
    const char* app = dir_app();
    if (app && *app)
    {
        const ct::String beside = joined(ct::String(app), "icons");
        if (isDirectory(beside)) return beside;
    }
#ifdef PRISMA3D_EDITOR_ICONS_SOURCE_DIR
    return ct::String(PRISMA3D_EDITOR_ICONS_SOURCE_DIR);
#else
    return ct::String();
#endif
}

bool EditorIcons::loadDefault()
{
    return load(defaultDirectory().c_str());
}

bool EditorIcons::loaded()
{
    ensureLoaded();
    return state().loaded;
}

unsigned int EditorIcons::generation()
{
    return state().generation;
}

rt::IconId EditorIcons::get(const char* name)
{
    ensureLoaded();
    const State& current = state();
    if (!current.loaded || !name) return rt::IconId::None;
    const std::size_t* found = current.index.find(ct::String(name));
    return found ? current.entries[*found].icon : rt::IconId::None;
}

rt::IconId EditorIcons::get(const ct::String& name)
{
    return get(name.c_str());
}

bool EditorIcons::has(const char* name)
{
    return get(name) != rt::IconId::None;
}

std::size_t EditorIcons::count()
{
    return state().entries.size();
}

const ct::String& EditorIcons::nameAt(std::size_t index)
{
    static const ct::String empty;
    const State& current = state();
    return index < current.entries.size() ? current.entries[index].name : empty;
}

rt::IconId EditorIcons::idAt(std::size_t index)
{
    const State& current = state();
    return index < current.entries.size() ? current.entries[index].icon : rt::IconId::None;
}

rt::Rect EditorIcons::uvOf(const char* name)
{
    const State& current = state();
    const std::size_t* found = name ? current.index.find(ct::String(name)) : nullptr;
    return found ? current.entries[*found].uv : rt::Rect{ 0.0f, 0.0f, 0.0f, 0.0f };
}

const ct::String& EditorIcons::directory()
{
    return state().directory;
}

} // namespace prisma3d
