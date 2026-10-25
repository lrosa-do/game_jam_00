#include "render3d.h"

#include "renderer.h"

#include <GL/gl.h>

#include <math.h>

namespace testbed
{

using namespace kinetix::Physics;
namespace M = kinetix::Math;
using kinetix::usize;

void setupLighting3D()
{
    // Fixed-function lighting: one directional key light plus ambient.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);

    const GLfloat direction[4] = {0.4f, 1.0f, 0.6f, 0.0f};
    const GLfloat diffuse[4] = {0.85f, 0.85f, 0.82f, 1.0f};
    const GLfloat ambient[4] = {0.35f, 0.36f, 0.40f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, direction);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
}

namespace
{

void endShading()
{
    glDisable(GL_LIGHTING);
}

void beginShading()
{
    glEnable(GL_LIGHTING);
}

} // namespace

void drawSolidBox(const M::mat4& transform, const M::vec3& halfExtents, Shade shade)
{
    beginShading();
    glPushMatrix();
    glMultMatrixf(&transform[0][0]);
    glColor3f(shade.r, shade.g, shade.b);

    const float x = halfExtents.x;
    const float y = halfExtents.y;
    const float z = halfExtents.z;

    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glVertex3f(-x, -y, z); glVertex3f(x, -y, z); glVertex3f(x, y, z); glVertex3f(-x, y, z);
    glNormal3f(0, 0, -1);
    glVertex3f(-x, -y, -z); glVertex3f(-x, y, -z); glVertex3f(x, y, -z); glVertex3f(x, -y, -z);
    glNormal3f(0, 1, 0);
    glVertex3f(-x, y, -z); glVertex3f(-x, y, z); glVertex3f(x, y, z); glVertex3f(x, y, -z);
    glNormal3f(0, -1, 0);
    glVertex3f(-x, -y, -z); glVertex3f(x, -y, -z); glVertex3f(x, -y, z); glVertex3f(-x, -y, z);
    glNormal3f(1, 0, 0);
    glVertex3f(x, -y, -z); glVertex3f(x, y, -z); glVertex3f(x, y, z); glVertex3f(x, -y, z);
    glNormal3f(-1, 0, 0);
    glVertex3f(-x, -y, -z); glVertex3f(-x, -y, z); glVertex3f(-x, y, z); glVertex3f(-x, y, -z);
    glEnd();

    // Darker edges on top so adjacent same-coloured boxes do not merge.
    endShading();
    glColor3f(shade.r * 0.35f, shade.g * 0.35f, shade.b * 0.35f);
    static constexpr int edges[12][6] = {
        {-1,-1,-1, 1,-1,-1}, {1,-1,-1, 1,1,-1}, {1,1,-1, -1,1,-1}, {-1,1,-1, -1,-1,-1},
        {-1,-1,1, 1,-1,1}, {1,-1,1, 1,1,1}, {1,1,1, -1,1,1}, {-1,1,1, -1,-1,1},
        {-1,-1,-1, -1,-1,1}, {1,-1,-1, 1,-1,1}, {1,1,-1, 1,1,1}, {-1,1,-1, -1,1,1}};
    glBegin(GL_LINES);
    for (const auto& e : edges)
    {
        glVertex3f(float(e[0]) * x, float(e[1]) * y, float(e[2]) * z);
        glVertex3f(float(e[3]) * x, float(e[4]) * y, float(e[5]) * z);
    }
    glEnd();

    glPopMatrix();
}

void drawSolidSphere(const M::vec3& centre, float radius, Shade shade)
{
    beginShading();
    glPushMatrix();
    glTranslatef(centre.x, centre.y, centre.z);
    glColor3f(shade.r, shade.g, shade.b);

    constexpr int kStacks = 10;
    constexpr int kSlices = 16;
    for (int i = 0; i < kStacks; ++i)
    {
        const float lat0 = 3.14159265f * (-0.5f + float(i) / float(kStacks));
        const float lat1 = 3.14159265f * (-0.5f + float(i + 1) / float(kStacks));
        const float y0 = sinf(lat0), r0 = cosf(lat0);
        const float y1 = sinf(lat1), r1 = cosf(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= kSlices; ++j)
        {
            const float lng = 2.0f * 3.14159265f * float(j) / float(kSlices);
            const float xs = cosf(lng), zs = sinf(lng);
            glNormal3f(xs * r0, y0, zs * r0);
            glVertex3f(xs * r0 * radius, y0 * radius, zs * r0 * radius);
            glNormal3f(xs * r1, y1, zs * r1);
            glVertex3f(xs * r1 * radius, y1 * radius, zs * r1 * radius);
        }
        glEnd();
    }

    glPopMatrix();
    endShading();
}

void drawGround(float y, float extent, float spacing)
{
    // The plane is pushed back in depth: a fixed y offset z-fights at all but one distance.
    beginShading();
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.30f, 0.32f, 0.35f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-extent, y, -extent);
    glVertex3f(-extent, y, extent);
    glVertex3f(extent, y, extent);
    glVertex3f(extent, y, -extent);
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);
    endShading();

    glColor3f(0.42f, 0.44f, 0.48f);
    glBegin(GL_LINES);
    for (float t = -extent; t <= extent + 0.001f; t += spacing)
    {
        glVertex3f(t, y, -extent);
        glVertex3f(t, y, extent);
        glVertex3f(-extent, y, t);
        glVertex3f(extent, y, t);
    }
    glEnd();
}

Shade shadeForBody(const RigidBody& body)
{
    if (body.bodyType() == BodyType::Static)
        return Shade{0.55f, 0.56f, 0.58f};
    if (body.bodyType() == BodyType::Kinematic)
        return Shade{0.35f, 0.62f, 0.90f};
    if (!body.awake())
        return Shade{0.42f, 0.52f, 0.45f};
    return Shade{0.45f, 0.80f, 0.55f};
}

void drawSolidConvexHull(const M::mat4& transform, const ConvexHullShape& hull, Shade shade)
{
    using Edge = ConvexHullShape::Edge;
    const ct::Vector<M::vec3>& verts = hull.vertices();
    const ct::Vector<Edge>& edges = hull.edges();
    const ct::Vector<int>& faces = hull.faces();

    beginShading();
    glPushMatrix();
    glMultMatrixf(&transform[0][0]);
    glColor3f(shade.r, shade.g, shade.b);

    // Each face's edge ring via getNextEdgeOfFace() (VoronoiShatter.cpp's convention); a flat normal
    // per face is enough for small debris.
    for (usize f = 0; f < faces.size(); ++f)
    {
        const Edge* start = &edges[static_cast<usize>(faces[f])];
        const M::vec3& a = verts[static_cast<usize>(start->getSourceVertex())];
        const Edge* e = start->getNextEdgeOfFace();
        const M::vec3& b = verts[static_cast<usize>(e->getSourceVertex())];
        const M::vec3 normal = M::normalize(M::cross(b - a, verts[static_cast<usize>(
                                                         e->getNextEdgeOfFace()->getSourceVertex())] -
                                                     a));

        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(a.x, a.y, a.z);
        const Edge* walk = e;
        do
        {
            const M::vec3& v = verts[static_cast<usize>(walk->getSourceVertex())];
            glVertex3f(v.x, v.y, v.z);
            walk = walk->getNextEdgeOfFace();
        } while (walk != start);
        glEnd();
    }

    endShading();
    glColor3f(shade.r * 0.35f, shade.g * 0.35f, shade.b * 0.35f);
    glBegin(GL_LINES);
    for (usize i = 0; i < edges.size(); ++i)
    {
        const M::vec3& a = verts[static_cast<usize>(edges[i].getSourceVertex())];
        const M::vec3& b = verts[static_cast<usize>(edges[i].getTargetVertex())];
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
    }
    glEnd();

    glPopMatrix();
}

void drawWorldSolid(const PhysicsWorld& world, const ct::Vector<RigidBody*>& bodies,
                    const ct::Vector<CollisionShape*>& shapes)
{
    (void)world;
    for (usize i = 0; i < bodies.size() && i < shapes.size(); ++i)
    {
        const RigidBody* body = bodies[i];
        const CollisionShape* shape = shapes[i];
        if (!body || !shape)
            continue;

        const Shade shade = shadeForBody(*body);
        if (const BoxShape* box = dynamic_cast<const BoxShape*>(shape))
            drawSolidBox(body->transform(), box->halfExtents(), shade);
        else if (const SphereShape* sphere = dynamic_cast<const SphereShape*>(shape))
            drawSolidSphere(body->position(), sphere->radius(), shade);
        else if (const ConvexHullShape* hull = dynamic_cast<const ConvexHullShape*>(shape))
            drawSolidConvexHull(body->transform(), *hull, shade);
    }
}

} // namespace testbed
