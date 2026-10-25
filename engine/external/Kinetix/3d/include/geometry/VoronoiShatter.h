#ifndef KINETIX_GEOMETRY_VORONOI_SHATTER_H
#define KINETIX_GEOMETRY_VORONOI_SHATTER_H

#include "ConvexHullComputer.h"
#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::Geometry
{

struct Shard
{
    ct::Vector<Math::vec3> vertices;
    ct::Vector<ConvexHullComputer::Edge> edges;
    ct::Vector<int> faces;
    Math::vec3 centroid = Math::vec3(0.0f);
    f32 volume = 0.0f;
};

// Voronoi cell decomposition of a convex point cloud (Stan Melax volume
// integration for mass properties, Preparata & Hong for the cell hulls).
class VoronoiShatter
{
public:
    // planes: half-space equations, xyz = normal, w = offset; a point p is inside
    // when dot(normal, p) + w <= 0 for every plane. Returns the vertices formed by
    // every triple-plane intersection that lies inside all planes, together with
    // the (sorted, unique) indices of the planes that actually produced one of
    // those vertices. Returns false when no such vertex exists.
    static bool getVerticesInsidePlanes(const ct::Vector<Math::vec4>& planes,
                                         ct::Vector<Math::vec3>& verticesOut,
                                         ct::Vector<int>& planeIndicesOut);

    // sourceVertices: convex point cloud of the shape being shattered, in its
    // local space. voronoiPoints: cell sites in that space; sites that
    // coincide (within 1e-5) are one site. Produces one Shard per distinct
    // site whose cell encloses volume; each shard's vertices are relative to
    // its own centroid (Shard::centroid is that centroid in the shared local
    // space).
    static void shatter(const ct::Vector<Math::vec3>& sourceVertices,
                         const ct::Vector<Math::vec3>& voronoiPoints,
                         ct::Vector<Shard>& shardsOut);
};

} // namespace kinetix::Geometry

#endif // KINETIX_GEOMETRY_VORONOI_SHATTER_H
