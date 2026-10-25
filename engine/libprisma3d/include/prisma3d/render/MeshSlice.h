#pragma once

#include "prisma3d/render/MeshStore.h"

#include <ct/vector.hpp>

namespace prisma3d
{

// A point of a polygon laid on a plane.
struct PolygonPoint
{
    float x = 0.0f;
    float y = 0.0f;
};

// Cuts a simple polygon into triangles between its own corners (ear clipping); the triangles index
// the polygon and run counter-clockwise. False when the polygon folds over itself.
bool triangulatePolygon(const ct::Vector<PolygonPoint>& polygon, ct::Vector<int>& triangles);

// Keeps the part of a triangle surface behind a plane (where dot(normal, p) + offset <= 0): the
// triangles across the plane are cut on it, the new corners take their attributes from the cut
// edge, and every ring the cut leaves open is closed with a cap facing along the normal (unless
// cap is false). Skin weights are not kept: the result is a static surface. False when nothing of
// the surface is behind the plane.
bool sliceSurface(const SurfaceData& in, const Math::Vec3& normal, float offset, SurfaceData& out,
        bool cap = true);

} // namespace prisma3d
