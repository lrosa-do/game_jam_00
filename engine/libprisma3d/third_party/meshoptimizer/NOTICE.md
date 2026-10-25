# Vendored: meshoptimizer

Used by `libprisma3d/src/prisma3d/render/MeshStore.cpp` to generate vertex tangents and the
levels of detail of a surface.

- `meshoptimizer.h`, `tangentspace.cpp`, `simplifier.cpp`, `allocator.cpp` and `LICENSE.md` — from meshoptimizer 1.2
  (https://github.com/zeux/meshoptimizer, commit `9d9890c73011d75920af614485296d1e03e95448`).
  MIT license — see `LICENSE.md`.

The files are not modified. Only the tangent generator and the simplifier are compiled; the other
sources of the library are not copied.
