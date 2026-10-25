#include "kx/world.h"

namespace kx
{

    namespace
    {

        int ClipPolygonToHalfPlane(const vec2 *in, int inCount,
                                   const vec2 &point, const vec2 &normal,
                                   vec2 *out, int maxOut)
        {
            int outCount = 0;
            for (int i = 0; i < inCount; ++i)
            {
                vec2 a = in[i];
                vec2 b = in[(i + 1) % inCount];
                float da = Dot(a - point, normal);
                float db = Dot(b - point, normal);

                if (da >= 0.0f)
                {
                    if (outCount < maxOut)
                        out[outCount++] = a;
                    if (db < 0.0f && outCount < maxOut)
                    {
                        float t = da / (da - db);
                        out[outCount++] = a + t * (b - a);
                    }
                }
                else if (db >= 0.0f)
                {
                    if (outCount < maxOut)
                    {
                        float t = da / (da - db);
                        out[outCount++] = a + t * (b - a);
                    }
                }
            }
            return outCount;
        }

        struct SlicePiece
        {
            ShapeType type;
            Circle circle;
            Edge edge;
            Polygon polygon;
            float density;
            Filter filter;
            bool isSensor;
            void *userData;
        };

        // A convex n-gon cut by a line can have n + 1 vertices, one more than a polygon holds,
        // so the clipped polygon is added in as many pieces as it takes. Slivers are skipped.
        void AddClippedPolygon(const vec2 *vertices, int count, const Shape &shape, ct::Vector<SlicePiece> &out)
        {
            if (count < 3)
                return;
            if (count > kMaxPolygonVertices)
            {
                // Fan split of a convex polygon: [0..half] and [0, half..n).
                const int half = count / 2;
                vec2 first[kMaxPolygonVertices + 2];
                vec2 second[kMaxPolygonVertices + 2];
                int firstCount = 0;
                int secondCount = 0;
                for (int i = 0; i <= half; ++i)
                    first[firstCount++] = vertices[i];
                second[secondCount++] = vertices[0];
                for (int i = half; i < count; ++i)
                    second[secondCount++] = vertices[i];
                AddClippedPolygon(first, firstCount, shape, out);
                AddClippedPolygon(second, secondCount, shape, out);
                return;
            }

            SlicePiece piece;
            piece.type = ShapeType::Polygon;
            if (!piece.polygon.Set(vertices, count))
                return;
            piece.density = shape.density;
            piece.filter = shape.filter;
            piece.isSensor = shape.isSensor;
            piece.userData = shape.userData;
            out.push_back(piece);
        }

        Body *BuildSlicedBody(World &world, const Body &source, ct::Vector<SlicePiece> &pieces)
        {
            Body *body = world.CreateBody(source.Type(), source.Position(), source.Angle());
            body->SetFriction(source.Friction());
            body->SetRestitution(source.Restitution());
            body->SetLinearDamping(source.LinearDamping());
            body->SetAngularDamping(source.AngularDamping());
            body->SetGravityScale(source.GravityScale());
            body->SetFixedRotation(source.FixedRotation());
            body->SetBullet(source.IsBullet());
            body->SetUserData(source.UserData());

            for (size_t i = 0; i < pieces.size(); ++i)
            {
                SlicePiece &p = pieces[i];
                int shapeIndex = body->ShapeCount();
                int added = 0;
                switch (p.type)
                {
                case ShapeType::Circle:
                    added = body->AddCircle(p.circle.center, p.circle.radius, p.density);
                    break;
                case ShapeType::Polygon:
                    added = body->AddPolygon(p.polygon.vertices, p.polygon.count, p.density);
                    break;
                case ShapeType::Edge:
                default:
                    // The whole edge, so chain links stay one-sided with
                    // their ghost vertices.
                    added = body->AddEdge(p.edge);
                    break;
                }
                if (!added)
                    continue;

                body->SetShapeFilter(shapeIndex, p.filter.category, p.filter.mask, p.filter.group);
                body->SetSensor(shapeIndex, p.isSensor);
                body->SetShapeUserData(shapeIndex, p.userData);
            }

            // Rigid-body velocity at the piece's own centre of mass.
            const vec2 offset = body->WorldCenter() - source.WorldCenter();
            body->SetVelocity(source.Velocity() + Cross(source.AngularVelocity(), offset));
            body->SetAngularVelocity(source.AngularVelocity());

            return body;
        }
    } 

    bool Slice(World &world, Body *body, const vec2 &point, const vec2 &normal,
              Body **outPositive, Body **outNegative, float separationSpeed)
    {
        if (outPositive)
            *outPositive = nullptr;
        if (outNegative)
            *outNegative = nullptr;

        if (!body || body->IsDestroyPending())
            return false;

        vec2 n = Normalize(normal);
        if (Dot(n, n) < 0.5f) 
            return false;

        Transform xf = body->GetTransform();
        vec2 localPoint = InvTransformPoint(xf, point);

        vec2 localNormal = InvRotate(xf, n);

        ct::Vector<SlicePiece> positive;
        ct::Vector<SlicePiece> negative;

        const int kClipMax = kMaxPolygonVertices + 2;
        vec2 posBuf[kClipMax];
        vec2 negBuf[kClipMax];

        for (int s = 0; s < body->ShapeCount(); ++s)
        {
            const Shape &shape = body->Shapes()[s];

            if (shape.type == ShapeType::Polygon)
            {
                const Polygon &poly = shape.polygon;
                int posCount = ClipPolygonToHalfPlane(poly.vertices, poly.count, localPoint, localNormal, posBuf, kClipMax);
                int negCount = ClipPolygonToHalfPlane(poly.vertices, poly.count, localPoint, -localNormal, negBuf, kClipMax);

                AddClippedPolygon(posBuf, posCount, shape, positive);
                AddClippedPolygon(negBuf, negCount, shape, negative);
                continue;
            }

            vec2 mid = shape.type == ShapeType::Circle
                                ? shape.circle.center
                                : 0.5f * (shape.edge.vertex1 + shape.edge.vertex2);
            float d = Dot(mid - localPoint, localNormal);

            SlicePiece piece;
            piece.type = shape.type;
            piece.circle = shape.circle;
            piece.edge = shape.edge;
            piece.density = shape.density;
            piece.filter = shape.filter;
            piece.isSensor = shape.isSensor;
            piece.userData = shape.userData;
            (d >= 0.0f ? positive : negative).push_back(piece);
        }

        if (positive.empty() || negative.empty())
            return false; 
        if ((int)positive.size() > Body::kMaxShapes || (int)negative.size() > Body::kMaxShapes)
            return false;

        Body *posBody = BuildSlicedBody(world, *body, positive);
        Body *negBody = BuildSlicedBody(world, *body, negative);

        if (separationSpeed > 0.0f)
        {
            posBody->SetVelocity(posBody->Velocity() + n * separationSpeed);
            negBody->SetVelocity(negBody->Velocity() - n * separationSpeed);
        }

        world.Destroy(body);

        if (outPositive)
            *outPositive = posBody;
        if (outNegative)
            *outNegative = negBody;
        return true;
    }

} 
