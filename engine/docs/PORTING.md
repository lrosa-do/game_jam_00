# Porting notes

Where each technique comes from. Paths are relative to the reference engine checkout.

| prisma3d | Origin |
|---|---|
| `render/MeshSlice.cpp` `triangulatePolygon` (ear clipping) | `core/math/triangulate.cpp:47-190` (`is_inside_triangle`, `snip`, `triangulate`) |
| `render/MeshSlice.cpp` `sliceSurface` (triangle against plane) | `core/math/geometry_3d.h:485` (`clip_polygon`) |
| `scene/Dismember3D.cpp` cap triangulation | `core/math/triangulate.cpp:114` (`triangulate`) |
