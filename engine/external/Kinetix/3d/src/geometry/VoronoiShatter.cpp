#include "geometry/VoronoiShatter.h"
#include <kinetix/core/Algorithm.h>

#include <ct/sort.hpp>
#include <math.h>

namespace kinetix::Geometry
{

namespace
{

constexpr f32 kInfinity = HUGE_VALF;
constexpr f32 kCrossEpsilon = 0.0001f;
constexpr f32 kQuotientEpsilon = 0.0001f;
constexpr f32 kInsideEpsilon = 0.000001f;
constexpr f32 kSiteEpsilon = 0.00001f;
// Cell volume * 6, relative to the cube of its size, below which the cell is flat.
constexpr f32 kVolumeEpsilon = 0.000001f;

void insertSorted(ct::Vector<int>& values, int value)
{
    ct::Vector<int>::iterator it = kinetix::LowerBound(values.begin(), values.end(), value);
    if (it == values.end() || *it != value)
    {
        values.insert(it, value);
    }
}

f32 triple(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c)
{
    return Math::dot(a, Math::cross(b, c));
}

struct DistanceFromPoint
{
    Math::vec3 center;

    bool operator()(const Math::vec3& a, const Math::vec3& b) const
    {
        f32 da = Math::dot(a - center, a - center);
        f32 db = Math::dot(b - center, b - center);
        return da < db;
    }
};

} // namespace

bool VoronoiShatter::getVerticesInsidePlanes(const ct::Vector<Math::vec4>& planes,
                                              ct::Vector<Math::vec3>& verticesOut,
                                              ct::Vector<int>& planeIndicesOut)
{
    verticesOut.clear();
    planeIndicesOut.clear();
    const int numPlanes = (int)planes.size();
    for (int i = 0; i < numPlanes; i++)
    {
        Math::vec3 n1(planes[i]);
        for (int j = i + 1; j < numPlanes; j++)
        {
            Math::vec3 n2(planes[j]);
            Math::vec3 n1n2 = Math::cross(n1, n2);
            if (Math::dot(n1n2, n1n2) > kCrossEpsilon)
            {
                for (int k = j + 1; k < numPlanes; k++)
                {
                    Math::vec3 n3(planes[k]);
                    Math::vec3 n2n3 = Math::cross(n2, n3);
                    Math::vec3 n3n1 = Math::cross(n3, n1);
                    if ((Math::dot(n2n3, n2n3) > kCrossEpsilon) && (Math::dot(n3n1, n3n1) > kCrossEpsilon))
                    {
                        f32 quotient = Math::dot(n1, n2n3);
                        if (fabsf(quotient) > kQuotientEpsilon)
                        {
                            Math::vec3 potentialVertex = (n2n3 * planes[i].w + n3n1 * planes[j].w + n1n2 * planes[k].w) * (-1.0f / quotient);
                            int l = 0;
                            for (; l < numPlanes; l++)
                            {
                                const Math::vec4& np = planes[l];
                                if (Math::dot(Math::vec3(np), potentialVertex) + np.w > kInsideEpsilon)
                                {
                                    break;
                                }
                            }
                            if (l == numPlanes)
                            {
                                verticesOut.push_back(potentialVertex);
                                insertSorted(planeIndicesOut, i);
                                insertSorted(planeIndicesOut, j);
                                insertSorted(planeIndicesOut, k);
                            }
                        }
                    }
                }
            }
        }
    }
    return !verticesOut.empty();
}

void VoronoiShatter::shatter(const ct::Vector<Math::vec3>& sourceVertices,
                              const ct::Vector<Math::vec3>& voronoiPoints,
                              ct::Vector<Shard>& shardsOut)
{
    shardsOut.clear();

    ConvexHullComputer sourceHull;
    if (!sourceVertices.empty())
    {
        sourceHull.compute(&sourceVertices[0].x, sizeof(Math::vec3), (int)sourceVertices.size(), 0.0f, 0.0f);
    }

    ct::Vector<Math::vec4> convexPlanes;
    const int numSourceFaces = (int)sourceHull.faces.size();
    for (int i = 0; i < numSourceFaces; i++)
    {
        const ConvexHullComputer::Edge* edge = &sourceHull.edges[sourceHull.faces[i]];
        int v0 = edge->getSourceVertex();
        int v1 = edge->getTargetVertex();
        edge = edge->getNextEdgeOfFace();
        int v2 = edge->getTargetVertex();
        Math::vec3 normal = Math::normalize(Math::cross(sourceHull.vertices[v1] - sourceHull.vertices[v0],
                                                       sourceHull.vertices[v2] - sourceHull.vertices[v0]));
        f32 offset = -Math::dot(normal, sourceHull.vertices[v0]);
        convexPlanes.push_back(Math::vec4(normal, offset));
    }
    const int numConvexPlanes = (int)convexPlanes.size();

    // Coincident sites are merged first: two sites at one point have no
    // bisector (zero-length normal, NaN plane) and both cells would grow to
    // the whole region the pair owned.
    ct::Vector<Math::vec3> sites;
    sites.reserve(voronoiPoints.size());
    for (const Math::vec3& point : voronoiPoints)
    {
        bool duplicate = false;
        for (const Math::vec3& site : sites)
        {
            const Math::vec3 delta = point - site;
            if (Math::dot(delta, delta) <= kSiteEpsilon * kSiteEpsilon)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            sites.push_back(point);
    }

    ct::Vector<Math::vec3> sortedVoronoiPoints(sites);
    const int numPoints = (int)sites.size();

    ct::Vector<Math::vec3> vertices;
    ct::Vector<Math::vec4> planes;
    ct::Vector<int> planeIndices;

    for (int i = 0; i < numPoints; i++)
    {
        const Math::vec3 curVoronoiPoint = sites[i];

        planes = convexPlanes;
        for (int j = 0; j < numConvexPlanes; j++)
        {
            planes[j].w += Math::dot(Math::vec3(planes[j]), curVoronoiPoint);
        }

        f32 maxDistance = kInfinity;
        DistanceFromPoint distanceFromPoint;
        distanceFromPoint.center = curVoronoiPoint;
        ct::sort(sortedVoronoiPoints.begin(), sortedVoronoiPoints.end(), distanceFromPoint);

        // Seed "vertices" from the source shape's own planes: the loop below
        // only populates them when adding a bisector against another site, so
        // a lone site would otherwise be dropped.
        getVerticesInsidePlanes(planes, vertices, planeIndices);
        for (int j = 1; j < numPoints; j++)
        {
            Math::vec3 normal = sortedVoronoiPoints[j] - curVoronoiPoint;
            f32 nlength = Math::length(normal);
            if (nlength > maxDistance)
            {
                break;
            }
            // Merged above, so only the site itself: never add a zero normal.
            if (!(nlength > kSiteEpsilon))
            {
                continue;
            }
            Math::vec4 plane(normal / nlength, -nlength * 0.5f);
            planes.push_back(plane);
            getVerticesInsidePlanes(planes, vertices, planeIndices);
            if (vertices.empty())
            {
                break;
            }
            int numPlaneIndices = (int)planeIndices.size();
            if (numPlaneIndices != (int)planes.size())
            {
                for (int k = 0; k < numPlaneIndices; k++)
                {
                    if (k != planeIndices[k])
                    {
                        planes[k] = planes[planeIndices[k]];
                    }
                }
                planes.resize(numPlaneIndices);
            }
            maxDistance = Math::length(vertices[0]);
            for (int k = 1; k < (int)vertices.size(); k++)
            {
                f32 distance = Math::length(vertices[k]);
                if (maxDistance < distance)
                {
                    maxDistance = distance;
                }
            }
            maxDistance *= 2.0f;
        }
        if (vertices.empty())
        {
            continue;
        }

        ConvexHullComputer cellHull;
        cellHull.compute(&vertices[0].x, sizeof(Math::vec3), (int)vertices.size(), 0.0f, 0.0f);

        const int numFaces = (int)cellHull.faces.size();
        f32 volume = 0.0f;
        Math::vec3 com(0.0f);
        for (int j = 0; j < numFaces; j++)
        {
            const ConvexHullComputer::Edge* edge = &cellHull.edges[cellHull.faces[j]];
            int v0 = edge->getSourceVertex();
            int v1 = edge->getTargetVertex();
            edge = edge->getNextEdgeOfFace();
            int v2 = edge->getTargetVertex();
            while (v2 != v0)
            {
                f32 vol = triple(cellHull.vertices[v0], cellHull.vertices[v1], cellHull.vertices[v2]);
                volume += vol;
                com += vol * (cellHull.vertices[v0] + cellHull.vertices[v1] + cellHull.vertices[v2]);
                edge = edge->getNextEdgeOfFace();
                v1 = v2;
                v2 = edge->getTargetVertex();
            }
        }
        // A cell with no volume (flat, or a sliver) has no centre of mass to
        // divide out; skip it rather than produce a NaN centroid.
        f32 extent = 0.0f;
        for (const Math::vec3& vertex : cellHull.vertices)
        {
            extent = Math::max(extent, Math::length(vertex));
        }
        if (!(volume > kVolumeEpsilon * extent * extent * extent))
        {
            continue;
        }
        com /= volume * 4.0f;
        volume /= 6.0f;

        const int numVerts = (int)cellHull.vertices.size();
        for (int j = 0; j < numVerts; j++)
        {
            cellHull.vertices[j] -= com;
        }

        Shard shard;
        shard.vertices = kinetix::Move(cellHull.vertices);
        shard.edges = kinetix::Move(cellHull.edges);
        shard.faces = kinetix::Move(cellHull.faces);
        shard.centroid = curVoronoiPoint + com;
        shard.volume = volume;
        shardsOut.push_back(kinetix::Move(shard));
    }
}

} // namespace kinetix::Geometry
