# Vendored: stb_image / stb_image_write

Used by `libprisma3d/src/prisma3d/core/Pixmap.cpp` to load/save PNG/JPG/BMP/TGA.

- `stb_image.h`, `stb_image_write.h` — from https://github.com/nothings/stb,
  `master` branch, fetched 2026-08-29. Public domain (Unlicense) / MIT, at
  the author's option - see the license block at the end of each file.

Neither file is modified from upstream. Re-fetch from the URL above to
update. The implementation is compiled once, in `libprisma3d/third_party/stb/stb_impl.cpp`
(`STB_IMAGE_IMPLEMENTATION` / `STB_IMAGE_WRITE_IMPLEMENTATION`).
