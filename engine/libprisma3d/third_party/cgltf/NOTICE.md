# Vendored: cgltf

Used by `libprisma3d/src/prisma3d/import/GltfImporter.cpp` to parse the JSON of glTF and GLB files.

- `cgltf.h` — the single-header glTF parser (https://github.com/jkuhlmann/cgltf), taken from the
  copy in prisma-samples `common/third_party/cgltf.h` on 2026-10-09. MIT license — see the block at
  the top of the file.

The file is not modified. The implementation is compiled once, in `cgltf_impl.c`.
