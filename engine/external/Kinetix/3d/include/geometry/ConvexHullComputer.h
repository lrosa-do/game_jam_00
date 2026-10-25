#ifndef KINETIX_GEOMETRY_CONVEX_HULL_COMPUTER_H
#define KINETIX_GEOMETRY_CONVEX_HULL_COMPUTER_H

#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::Geometry
{

// Incremental 3D convex hull construction (Preparata & Hong).
class ConvexHullComputer
{
public:
    class Edge
    {
    public:
        int getSourceVertex() const;
        int getTargetVertex() const;
        const Edge* getNextEdgeOfVertex() const;
        const Edge* getNextEdgeOfFace() const;
        const Edge* getReverseEdge() const;

    private:
        int next;
        int reverse;
        int targetVertex;

        friend class ConvexHullComputer;
    };

    ct::Vector<Math::vec3> vertices;
    ct::Vector<Edge> edges;
    ct::Vector<int> faces;

    // Computes the convex hull of "count" vertices stored in "coords". "stride" is the
    // difference in bytes between the addresses of consecutive vertices. If "shrink" is
    // positive, the hull is shrunk by that amount (each face moved towards the center
    // along its normal). If "shrinkClamp" is positive, "shrink" is clamped to
    // "shrinkClamp * innerRadius", the minimum distance of a face to the center.
    //
    // Returns the amount the hull was shrunk; negative if that was so large the
    // resulting hull is empty.
    f32 compute(const float* coords, int stride, int count, f32 shrink, f32 shrinkClamp);
};

} // namespace kinetix::Geometry

#endif // KINETIX_GEOMETRY_CONVEX_HULL_COMPUTER_H
