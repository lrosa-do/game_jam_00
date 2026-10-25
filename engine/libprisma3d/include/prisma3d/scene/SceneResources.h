#pragma once

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

namespace sceneres
{

ct::String normalizePath(const ct::String& path);

bool alive(MeshId mesh);
bool alive(TextureId texture);
bool alive(MaterialId material);
bool alive(SoundId sound);

ct::String pathOf(MeshId mesh);
ct::String pathOf(TextureId texture);
ct::String pathOf(MaterialId material);
ct::String pathOf(SoundId sound);

MeshId loadMesh(const ct::String& path);
ModelId loadModel(const ct::String& path);
bool listModelMeshes(const ct::String& file, ct::Vector<ct::String>& paths);
bool reloadModel(const ct::String& file);
TextureId loadTexture(const ct::String& path, bool srgb);
MaterialId loadMaterial(const ct::String& path);
SoundId loadSound(const ct::String& path);

inline void load(const ct::String& path, bool srgb, MeshId& out)
{
    (void) srgb;
    out = loadMesh(path);
}

inline void load(const ct::String& path, bool srgb, TextureId& out)
{
    out = loadTexture(path, srgb);
}

inline void load(const ct::String& path, bool srgb, MaterialId& out)
{
    (void) srgb;
    out = loadMaterial(path);
}

inline void load(const ct::String& path, bool srgb, SoundId& out)
{
    (void) srgb;
    out = loadSound(path);
}

ct::String takeError();
void clearError();

} // namespace sceneres

} // namespace prisma3d
