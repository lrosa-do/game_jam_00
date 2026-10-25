#include "kx/body.h"
#include "kx/internal/triangulate.h"

#include <ct/vector.hpp>
#include <math.h>

namespace kx
{

    Body::Body()
        : mType(BodyType::Static),
          mPosition(0.0f, 0.0f), mAngle(0.0f),
          mLinearVelocity(0.0f, 0.0f), mAngularVelocity(0.0f),
          mSleepTime(0.0f), mAwake(true), mFixedRotation(false), mBullet(false),
          mInvMass(0.0f), mInvI(0.0f), mLocalCenter(0.0f, 0.0f),
          mFriction(0.2f), mRestitution(0.0f),
          mLinearDamping(0.0f), mAngularDamping(0.0f), mGravityScale(1.0f),
          mForce(0.0f, 0.0f), mTorque(0.0f), mId(0),
          mUserData(nullptr), mContactCallback(nullptr), mContactContext(nullptr),
          mProxyId(-1), mProxyPosition(0.0f, 0.0f),
          mTightAABB{vec2(0.0f), vec2(0.0f)},
          mShapeCount(0), mIslandIndex(-1), mDestroyPending(false)
    {
    }

    int Body::AddCircle(const vec2 &localCenter, float radius, float density)
    {
        if (mShapeCount >= kMaxShapes)
            return 0;

        Shape &shape = mShapes[mShapeCount++];
        shape.type = ShapeType::Circle;
        shape.density = density;
        shape.filter = mDefaultFilter;
        shape.isSensor = false;
        shape.userData = nullptr;
        shape.circle.center = localCenter;
        shape.circle.radius = radius;

        RecomputeMass();
        return 1;
    }

    int Body::AddBox(float halfWidth, float halfHeight, const vec2 &localCenter, float density)
    {
        if (mShapeCount >= kMaxShapes)
            return 0;

        Shape &shape = mShapes[mShapeCount++];
        shape.type = ShapeType::Polygon;
        shape.density = density;
        shape.filter = mDefaultFilter;
        shape.isSensor = false;
        shape.userData = nullptr;
        shape.polygon.SetAsBox(halfWidth, halfHeight, localCenter, 0.0f);

        RecomputeMass();
        return 1;
    }

    int Body::AddPolygon(const vec2 *points, int count, float density)
    {
        if (mShapeCount >= kMaxShapes || count < 3)
            return 0;

        Polygon polygon;
        if (!polygon.Set(points, count))
            return 0;

        Shape &shape = mShapes[mShapeCount++];
        shape.type = ShapeType::Polygon;
        shape.density = density;
        shape.filter = mDefaultFilter;
        shape.isSensor = false;
        shape.userData = nullptr;
        shape.polygon = polygon;

        RecomputeMass();
        return 1;
    }

    int Body::AddEdge(const vec2 &localA, const vec2 &localB)
    {
        if (mShapeCount >= kMaxShapes)
            return 0;

        Shape &shape = mShapes[mShapeCount++];
        shape.type = ShapeType::Edge;
        shape.density = 0.0f;
        shape.filter = mDefaultFilter;
        shape.isSensor = false;
        shape.userData = nullptr;
        shape.edge.SetTwoSided(localA, localB);

        RecomputeMass();
        return 1;
    }

    int Body::AddEdge(const Edge &edge)
    {
        if (mShapeCount >= kMaxShapes)
            return 0;

        Shape &shape = mShapes[mShapeCount++];
        shape.type = ShapeType::Edge;
        shape.density = 0.0f;
        shape.filter = mDefaultFilter;
        shape.isSensor = false;
        shape.userData = nullptr;
        shape.edge = edge;

        RecomputeMass();
        return 1;
    }

    int Body::AddChain(const vec2 *points, int count, bool loop)
    {
        int minCount = loop ? 3 : 2;
        if (count < minCount)
            return 0;

        int segments = loop ? count : count - 1;
        int added = 0;

        for (int i = 0; i < segments && mShapeCount < kMaxShapes; ++i)
        {
            int i1 = i;
            int i2 = (i + 1) % count;
            vec2 v1 = points[i1];
            vec2 v2 = points[i2];

            vec2 vPrev = v1;
            vec2 vNext = v2;
            if (loop || i > 0)
                vPrev = points[(i1 - 1 + count) % count];
            if (loop || i + 2 < count)
                vNext = points[(i2 + 1) % count];

            Shape &shape = mShapes[mShapeCount++];
            shape.type = ShapeType::Edge;
            shape.density = 0.0f;
            shape.filter = mDefaultFilter;
            shape.isSensor = false;
            shape.userData = nullptr;
            shape.edge.vertex1 = v1;
            shape.edge.vertex2 = v2;
            shape.edge.vertex0 = vPrev;
            shape.edge.vertex3 = vNext;
            shape.edge.radius = kPolygonRadius;
            shape.edge.oneSided = true;

            ++added;
        }

        if (added > 0)
            RecomputeMass();
        return added;
    }

    namespace
    {

        // A convex piece of a triangulated outline, counter-clockwise.
        struct MeshPiece
        {
            vec2 v[kMaxPolygonVertices];
            int count;
        };

        bool TryMerge(const MeshPiece &P, const MeshPiece &Q, MeshPiece &result)
        {
            if (P.count + Q.count - 2 > kMaxPolygonVertices)
                return false;

            for (int i = 0; i < P.count; ++i)
            {
                const vec2 a = P.v[i];
                const vec2 b = P.v[(i + 1) % P.count];
                for (int j = 0; j < Q.count; ++j)
                {
                    if (!(Q.v[j] == b && Q.v[(j + 1) % Q.count] == a))
                        continue;

                    // P from b round to a, then Q past a back to b.
                    result.count = 0;
                    for (int k = 0; k < P.count; ++k)
                        result.v[result.count++] = P.v[(i + 1 + k) % P.count];
                    for (int k = 2; k < Q.count; ++k)
                        result.v[result.count++] = Q.v[(j + k) % Q.count];

                    for (int k = 0; k < result.count; ++k)
                    {
                        const vec2 e1 = result.v[(k + 1) % result.count] - result.v[k];
                        const vec2 e2 = result.v[(k + 2) % result.count] - result.v[(k + 1) % result.count];
                        if (Cross(e1, e2) < -1.0e-5f * sqrtf(Dot(e1, e1) * Dot(e2, e2)))
                            return false;
                    }
                    return true;
                }
            }
            return false;
        }

        // Hertel-Mehlhorn greedy merge across shared edges while the result stays convex
        // and within kMaxPolygonVertices.
        void MergeConvexPieces(ct::Vector<MeshPiece> &pieces)
        {
            bool merged = true;
            while (merged)
            {
                merged = false;
                for (size_t p = 0; p < pieces.size(); ++p)
                {
                    for (size_t q = p + 1; q < pieces.size();)
                    {
                        MeshPiece result;
                        if (!TryMerge(pieces[p], pieces[q], result))
                        {
                            ++q;
                            continue;
                        }
                        pieces[p] = result;
                        pieces[q] = pieces.back();
                        pieces.pop_back();
                        merged = true;
                        q = p + 1;
                    }
                }
            }
        }

    } 

    int Body::AddMesh(const vec2 *outline, int count, float density)
    {
        const int added = AddMeshPieces(outline, count, nullptr, nullptr, 0, density);
        return added > 0 ? added : 0;
    }

    int Body::AddMeshPieces(const vec2 *outline, int count, const vec2 *const *holes, const int *holeCounts,
                            int holeCount, float density)
    {
        if (!outline || count < 3)
            return 0;

        // A simple polygon with n vertices and h holes of m_i vertices has
        // n + sum(m_i) + 2h - 2 triangles.
        int maxTriangles = count - 2;
        for (int h = 0; h < holeCount; ++h)
            maxTriangles += holeCounts[h] + 2;
        ct::Vector<vec2> triangles;
        triangles.resize((size_t)maxTriangles * 3);
        int triCount = TriangulateWithHoles(outline, count, holes, holeCounts, holeCount, triangles.data(),
                                            maxTriangles);
        if (triCount == 0)
            return -1;
        if (triCount < 0)
            return -2;

        ct::Vector<MeshPiece> pieces;
        pieces.reserve((size_t)triCount);
        for (int i = 0; i < triCount; ++i)
        {
            MeshPiece piece;
            piece.count = 3;
            piece.v[0] = triangles[(size_t)i * 3 + 0];
            piece.v[1] = triangles[(size_t)i * 3 + 1];
            piece.v[2] = triangles[(size_t)i * 3 + 2];
            if (Cross(piece.v[1] - piece.v[0], piece.v[2] - piece.v[0]) < 0.0f)
            {
                vec2 swap = piece.v[1];
                piece.v[1] = piece.v[2];
                piece.v[2] = swap;
            }
            pieces.push_back(piece);
        }
        MergeConvexPieces(pieces);

        // All or nothing: degenerate slivers are skipped, but the rest must fit.
        ct::Vector<Polygon> polygons;
        polygons.reserve(pieces.size());
        for (size_t i = 0; i < pieces.size(); ++i)
        {
            Polygon polygon;
            if (polygon.Set(pieces[i].v, pieces[i].count))
                polygons.push_back(polygon);
        }
        if (polygons.empty())
            return 0;
        if ((int)polygons.size() > kMaxShapes - mShapeCount)
            return -2;

        for (size_t i = 0; i < polygons.size(); ++i)
        {
            Shape &shape = mShapes[mShapeCount++];
            shape.type = ShapeType::Polygon;
            shape.density = density;
            shape.filter = mDefaultFilter;
            shape.isSensor = false;
            shape.userData = nullptr;
            shape.polygon = polygons[i];
        }

        RecomputeMass();
        return (int)polygons.size();
    }

    void Body::RecomputeMass()
    {
        mInvMass = 0.0f;
        mInvI = 0.0f;
        mLocalCenter = vec2(0.0f, 0.0f);

        if (mType != BodyType::Dynamic)
            return;

        float mass = 0.0f;
        vec2 center(0.0f, 0.0f);
        float I = 0.0f;

        for (int i = 0; i < mShapeCount; ++i)
        {
            const Shape &shape = mShapes[i];
            MassData massData;
            switch (shape.type)
            {
            case ShapeType::Circle:
                massData = shape.circle.ComputeMass(shape.density);
                break;
            case ShapeType::Polygon:
                massData = shape.polygon.ComputeMass(shape.density);
                break;
            case ShapeType::Edge:
            default:
                massData = shape.edge.ComputeMass(shape.density);
                break;
            }

            mass += massData.mass;
            center += massData.mass * massData.center;
            I += massData.I;
        }

        if (mass > 0.0f)
        {
            mInvMass = 1.0f / mass;
            center *= mInvMass;
        }
        else
        {
            mass = 1.0f;
            mInvMass = 1.0f;
        }

        if (I > 0.0f)
        {
            I -= mass * Dot(center, center);
            mInvI = I > 0.0f ? 1.0f / I : 0.0f;
        }

        if (mFixedRotation)
            mInvI = 0.0f;

        mLocalCenter = center;
    }

    void Body::IntegrateVelocity(const vec2 &gravity, float dt)
    {
        if (mType != BodyType::Dynamic)
            return;

        if (!mAwake)
        {

            mForce = vec2(0.0f);
            mTorque = 0.0f;
            return;
        }

        mLinearVelocity += dt * (mGravityScale * gravity + mInvMass * mForce);
        mAngularVelocity += dt * mInvI * mTorque;

        mLinearVelocity *= 1.0f / (1.0f + dt * mLinearDamping);
        mAngularVelocity *= 1.0f / (1.0f + dt * mAngularDamping);

        mForce = vec2(0.0f);
        mTorque = 0.0f;
    }

    void Body::IntegratePosition(float dt)
    {
        if (mType == BodyType::Static || !mAwake)
            return;

        vec2 translation = dt * mLinearVelocity;
        float translationSq = translation.x * translation.x + translation.y * translation.y;
        if (translationSq > kMaxTranslation * kMaxTranslation)
        {
            float ratio = kMaxTranslation / sqrtf(translationSq);
            mLinearVelocity *= ratio;
            translation *= ratio;
        }

        vec2 center = GetTransform().Transform(mLocalCenter);
        center += translation;
        mAngle += dt * mAngularVelocity;
        Transform xf = MakeTransform(vec2(0.0f, 0.0f), mAngle);
        mPosition = center - xf.Transform(mLocalCenter);
    }

    void Body::ShiftCenter(const vec2 &deltaCenter, float deltaAngle)
    {
        vec2 center = GetTransform().Transform(mLocalCenter);
        center += deltaCenter;
        mAngle += deltaAngle;
        Transform xf = MakeTransform(vec2(0.0f, 0.0f), mAngle);
        mPosition = center - xf.Transform(mLocalCenter);
    }

} 