#include "prisma3d/editor/EditorShapeGizmo.h"

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/Vehicle3D.h"

#include <math.h>

namespace prisma3d
{

using Math::Mat4;
using Math::Vec3;

namespace
{

const float kPi = 3.14159265358979f;
const int kCircleSegments = 48;
const float kPlaneExtent = 5.0f;
const int kPlaneLines = 4;
const float kPointCross = 0.05f;
const float kJointSize = 0.15f;
const float kJointAxis = 0.6f;
const float kJointRing = 0.3f;

void addLine(ct::Vector<Vec3>& lines, const Vec3& a, const Vec3& b)
{
    lines.push_back(a);
    lines.push_back(b);
}

void addCircle(ct::Vector<Vec3>& lines, const Vec3& centre, const Vec3& u, const Vec3& v,
        float radius, float fromAngle = 0.0f, float toAngle = 2.0f * kPi)
{
    const int segments = static_cast<int>(
            static_cast<float>(kCircleSegments) * (toAngle - fromAngle) / (2.0f * kPi) + 0.5f);
    const int count = segments > 1 ? segments : 1;
    Vec3 previous;
    for (int i = 0; i <= count; ++i)
    {
        const float angle = fromAngle + (toAngle - fromAngle) * static_cast<float>(i) / count;
        const Vec3 point = centre + u * (cosf(angle) * radius) + v * (sinf(angle) * radius);
        if (i > 0) addLine(lines, previous, point);
        previous = point;
    }
}

void addBox(ct::Vector<Vec3>& lines, const Vec3& half)
{
    const Vec3 c[8] = { Vec3(-half.x, -half.y, -half.z), Vec3(half.x, -half.y, -half.z),
        Vec3(half.x, half.y, -half.z), Vec3(-half.x, half.y, -half.z),
        Vec3(-half.x, -half.y, half.z), Vec3(half.x, -half.y, half.z), Vec3(half.x, half.y, half.z),
        Vec3(-half.x, half.y, half.z) };
    const int edges[12][2] = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 }, { 4, 5 }, { 5, 6 }, { 6, 7 },
        { 7, 4 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
    for (int i = 0; i < 12; ++i) addLine(lines, c[edges[i][0]], c[edges[i][1]]);
}

void addCross(ct::Vector<Vec3>& lines, const Vec3& p, float size)
{
    addLine(lines, p - Vec3(size, 0.0f, 0.0f), p + Vec3(size, 0.0f, 0.0f));
    addLine(lines, p - Vec3(0.0f, size, 0.0f), p + Vec3(0.0f, size, 0.0f));
    addLine(lines, p - Vec3(0.0f, 0.0f, size), p + Vec3(0.0f, 0.0f, size));
}

const PhysicsBody3D* bodyOf(const CollisionShape3D& shape)
{
    for (const Node* node = shape.getParent(); node; node = node->getParent())
        if (const PhysicsBody3D* body = node->castTo<PhysicsBody3D>()) return body;
    return nullptr;
}

} // namespace

EditorShapeGizmo::EditorShapeGizmo()
        : mShapes(0),
          mJoints(0),
          mWheels(0),
          mLines(0)
{
}

EditorShapeGizmo::Settings& EditorShapeGizmo::settings()
{
    return mSettings;
}

const EditorShapeGizmo::Settings& EditorShapeGizmo::settings() const
{
    return mSettings;
}

Vec3 EditorShapeGizmo::gizmoColor(const CollisionShape3D& shape)
{
    const PhysicsBody3D* body = bodyOf(shape);
    if (!body) return Vec3(0.7f, 0.7f, 0.7f);
    if (body->castTo<CharacterBody3D>()) return Vec3(0.35f, 0.9f, 0.45f);
    if (body->castTo<RigidBody3D>()) return Vec3(1.0f, 0.6f, 0.2f);
    return Vec3(0.45f, 0.7f, 1.0f);
}

Vec3 EditorShapeGizmo::jointColor()
{
    return Vec3(0.9f, 0.45f, 0.95f);
}

void EditorShapeGizmo::shape(const CollisionShape3D& shape, std::uint32_t maxConcaveEdges,
        ct::Vector<Vec3>& lines)
{
    lines.clear();
    const Vec3 x(1.0f, 0.0f, 0.0f);
    const Vec3 y(0.0f, 1.0f, 0.0f);
    const Vec3 z(0.0f, 0.0f, 1.0f);
    const Vec3 origin(0.0f, 0.0f, 0.0f);
    switch (shape.getKind())
    {
        case CollisionShapeKind::Box:
            addBox(lines, shape.getBoxSize() * 0.5f);
            break;
        case CollisionShapeKind::Sphere:
        {
            const float radius = shape.getSphereRadius();
            addCircle(lines, origin, x, y, radius);
            addCircle(lines, origin, x, z, radius);
            addCircle(lines, origin, y, z, radius);
            break;
        }
        case CollisionShapeKind::Capsule:
        {
            const float radius = shape.getCapsuleRadius();
            float half = shape.getCapsuleHeight() * 0.5f - radius;
            if (half < 0.0f) half = 0.0f;
            const Vec3 top(0.0f, half, 0.0f);
            const Vec3 bottom(0.0f, -half, 0.0f);
            addCircle(lines, top, x, z, radius);
            addCircle(lines, bottom, x, z, radius);
            addCircle(lines, top, x, y, radius, 0.0f, kPi);
            addCircle(lines, top, z, y, radius, 0.0f, kPi);
            addCircle(lines, bottom, x, y, radius, kPi, 2.0f * kPi);
            addCircle(lines, bottom, z, y, radius, kPi, 2.0f * kPi);
            addLine(lines, top + x * radius, bottom + x * radius);
            addLine(lines, top - x * radius, bottom - x * radius);
            addLine(lines, top + z * radius, bottom + z * radius);
            addLine(lines, top - z * radius, bottom - z * radius);
            break;
        }
        case CollisionShapeKind::Cylinder:
        {
            const float radius = shape.getCylinderRadius();
            const float half = shape.getCylinderHeight() * 0.5f;
            const Vec3 top(0.0f, half, 0.0f);
            const Vec3 bottom(0.0f, -half, 0.0f);
            addCircle(lines, top, x, z, radius);
            addCircle(lines, bottom, x, z, radius);
            addLine(lines, top + x * radius, bottom + x * radius);
            addLine(lines, top - x * radius, bottom - x * radius);
            addLine(lines, top + z * radius, bottom + z * radius);
            addLine(lines, top - z * radius, bottom - z * radius);
            break;
        }
        case CollisionShapeKind::Plane:
        {
            Vec3 normal = shape.getPlaneNormal();
            if (normal.LengthSquared() < 1.0e-8f) normal = y;
            normal = normal.Normalized();
            const Vec3 helper = fabsf(normal.y) < 0.9f ? y : x;
            const Vec3 u = helper.Cross(normal).Normalized();
            const Vec3 v = normal.Cross(u).Normalized();
            const Vec3 centre = normal * shape.getPlaneDistance();
            for (int i = -kPlaneLines; i <= kPlaneLines; ++i)
            {
                const float t = kPlaneExtent * static_cast<float>(i) / kPlaneLines;
                addLine(lines, centre + u * t - v * kPlaneExtent,
                        centre + u * t + v * kPlaneExtent);
                addLine(lines, centre + v * t - u * kPlaneExtent,
                        centre + v * t + u * kPlaneExtent);
            }
            addLine(lines, centre, centre + normal * (kPlaneExtent * 0.5f));
            break;
        }
        case CollisionShapeKind::ConvexHull:
        {
            ct::Vector<Vec3> points;
            if (!shape.getHullPoints(points) || points.empty()) break;
            Vec3 minimum = points[0];
            Vec3 maximum = points[0];
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                addCross(lines, points[i], kPointCross);
                minimum = Vec3(points[i].x < minimum.x ? points[i].x : minimum.x,
                        points[i].y < minimum.y ? points[i].y : minimum.y,
                        points[i].z < minimum.z ? points[i].z : minimum.z);
                maximum = Vec3(points[i].x > maximum.x ? points[i].x : maximum.x,
                        points[i].y > maximum.y ? points[i].y : maximum.y,
                        points[i].z > maximum.z ? points[i].z : maximum.z);
            }
            const Vec3 centre = (minimum + maximum) * 0.5f;
            const Vec3 half = (maximum - minimum) * 0.5f;
            ct::Vector<Vec3> box;
            addBox(box, half);
            for (std::size_t i = 0; i < box.size(); ++i) lines.push_back(box[i] + centre);
            break;
        }
        case CollisionShapeKind::ConcaveMesh:
        {
            PhysicsServer3D* server = PhysicsServer3D::singleton();
            if (!server || !server->shapeValid(shape.getShape())) break;
            ct::Vector<Vec3> vertices;
            ct::Vector<std::uint32_t> indices;
            if (!server->shapeGetConcaveFaces(shape.getShape(), vertices, indices)) break;
            std::uint32_t edges = 0;
            for (std::size_t i = 0; i + 2 < indices.size() && edges < maxConcaveEdges; i += 3)
            {
                const std::uint32_t a = indices[i];
                const std::uint32_t b = indices[i + 1];
                const std::uint32_t c = indices[i + 2];
                if (a >= vertices.size() || b >= vertices.size() || c >= vertices.size()) continue;
                addLine(lines, vertices[a], vertices[b]);
                addLine(lines, vertices[b], vertices[c]);
                addLine(lines, vertices[c], vertices[a]);
                edges += 3;
            }
            break;
        }
        default:
            break;
    }
}

void EditorShapeGizmo::collect(Node* node, ct::Vector<CollisionShape3D*>& shapes,
        ct::Vector<Joint3D*>& joints, ct::Vector<VehicleWheel3D*>& wheels) const
{
    if (!node) return;
    if (CollisionShape3D* shape = node->castTo<CollisionShape3D>()) shapes.push_back(shape);
    else if (Joint3D* joint = node->castTo<Joint3D>())
        joints.push_back(joint);
    else if (VehicleWheel3D* wheel = node->castTo<VehicleWheel3D>())
        wheels.push_back(wheel);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collect(node->getChild(i), shapes, joints, wheels);
}

void EditorShapeGizmo::drawJoint(BatchRenderer& batch, Joint3D* joint, float opacity)
{
    const Mat4 global = joint->getGlobalTransform();
    const Vec3 x = global[0].xyz().Normalized();
    const Vec3 y = global[1].xyz().Normalized();
    const Vec3 z = global[2].xyz().Normalized();
    const Vec3 position = global.GetTranslation();
    const Vec3 tint = jointColor();
    ct::Vector<Vec3> lines;

    const Vec3 axes[3] = { Vec3(kJointSize, 0.0f, 0.0f), Vec3(0.0f, kJointSize, 0.0f),
        Vec3(0.0f, 0.0f, kJointSize) };
    for (int a = 0; a < 3; ++a)
        for (int b = a + 1; b < 3; ++b)
        {
            const Vec3 corners[4] = { axes[a], axes[b], axes[a] * -1.0f, axes[b] * -1.0f };
            for (int k = 0; k < 4; ++k) addLine(lines, corners[k], corners[(k + 1) % 4]);
        }
    for (std::size_t i = 0; i < lines.size(); ++i) lines[i] = position + lines[i];

    Vec3 axis;
    Vec3 reference;
    bool rotational = false;
    bool linear = false;
    bool limited = false;
    float lower = 0.0f;
    float upper = 0.0f;
    if (joint->castTo<SliderJoint3D>() || joint->castTo<PistonJoint3D>())
    {
        axis = x;
        reference = y;
        linear = true;
        if (const SliderJoint3D* slider = joint->castTo<SliderJoint3D>())
        {
            limited = slider->getUseLimit();
            lower = slider->getLimitLower();
            upper = slider->getLimitUpper();
        }
        if (joint->castTo<PistonJoint3D>()) rotational = true;
    }
    else if (const UniversalJoint3D* universal = joint->castTo<UniversalJoint3D>())
    {
        (void) universal;
        addLine(lines, position - x * kJointAxis, position + x * kJointAxis);
        addLine(lines, position - y * kJointAxis, position + y * kJointAxis);
        addCircle(lines, position, y, z, kJointRing);
        addCircle(lines, position, x, z, kJointRing);
    }
    else if (joint->castTo<WheelJoint3D>())
    {
        axis = y * -1.0f;
        reference = x;
        linear = true;
        addCircle(lines, position, y, z, kJointRing);
    }
    else if (joint->castTo<HingeJoint3D>() || joint->castTo<PinJoint3D>() ||
             joint->castTo<ConeTwistJoint3D>())
    {
        axis = z;
        reference = x;
        rotational = true;
        if (const HingeJoint3D* hinge = joint->castTo<HingeJoint3D>())
        {
            limited = hinge->getUseLimit();
            lower = hinge->getLimitLower();
            upper = hinge->getLimitUpper();
        }
    }
    if (rotational || linear)
    {
        addLine(lines, position - axis * kJointAxis, position + axis * kJointAxis);
        const Vec3 side = axis.Cross(reference).Normalized();
        if (rotational)
        {
            if (limited && upper > lower)
                addCircle(lines, position, reference, side, kJointRing, lower, upper);
            else
                addCircle(lines, position, reference, side, kJointRing);
        }
        if (linear && limited && upper > lower)
        {
            const Vec3 tick = reference * (kJointSize * 0.5f);
            addLine(lines, position + axis * lower - tick, position + axis * lower + tick);
            addLine(lines, position + axis * upper - tick, position + axis * upper + tick);
        }
    }
    const PhysicsBody3D* bodies[2] = { joint->getBodyA(), joint->getBodyB() };
    for (int b = 0; b < 2; ++b)
        if (bodies[b]) addLine(lines, position, bodies[b]->getGlobalPosition());

    batch.setColor(tint.x, tint.y, tint.z, opacity);
    batch.begin(BatchRenderer::ModeLines);
    for (std::size_t i = 0; i < lines.size(); ++i)
        batch.vertex3(lines[i].x, lines[i].y, lines[i].z);
    batch.end();
    mLines += static_cast<std::uint32_t>(lines.size() / 2);
}

void EditorShapeGizmo::drawWheel(BatchRenderer& batch, VehicleWheel3D* wheel, float opacity)
{
    const Mat4 global = wheel->getGlobalTransform();
    const Vec3 x = global[0].xyz().Normalized();
    const Vec3 y = global[1].xyz().Normalized();
    const Vec3 z = global[2].xyz().Normalized();
    const Vec3 position = global.GetTranslation();
    const float radius = wheel->getRadius();
    const Vec3 down = y * -1.0f;
    const float rest = wheel->getSuspensionRestLength();
    const float travel = wheel->getSuspensionTravel();
    ct::Vector<Vec3> lines;
    const Vec3 hub = position + down * rest;
    addCircle(lines, hub, y, z, radius);
    addLine(lines, hub - x * (radius * 0.25f), hub + x * (radius * 0.25f));
    addLine(lines, position, hub);
    const Vec3 tick = x * (radius * 0.35f);
    addLine(lines, position - tick, position + tick);
    if (travel > 0.0f)
    {
        const Vec3 lowest = position + down * (rest + travel);
        addLine(lines, hub, lowest);
        addLine(lines, lowest - tick, lowest + tick);
        const Vec3 highest = position + down * (rest - travel > 0.0f ? rest - travel : 0.0f);
        addLine(lines, highest - tick, highest + tick);
    }
    batch.setColor(0.4f, 0.9f, 0.9f, opacity);
    batch.begin(BatchRenderer::ModeLines);
    for (std::size_t i = 0; i < lines.size(); ++i)
        batch.vertex3(lines[i].x, lines[i].y, lines[i].z);
    batch.end();
    mLines += static_cast<std::uint32_t>(lines.size() / 2);
}

void EditorShapeGizmo::draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
        const EditorView& view)
{
    (void) view;
    mShapes = 0;
    mJoints = 0;
    mWheels = 0;
    mLines = 0;
    ct::Vector<CollisionShape3D*> shapes;
    ct::Vector<Joint3D*> joints;
    ct::Vector<VehicleWheel3D*> wheels;
    collect(root, shapes, joints, wheels);
    if (!mSettings.shapes) shapes.clear();
    if (!mSettings.joints)
    {
        joints.clear();
        wheels.clear();
    }
    if (shapes.empty() && joints.empty() && wheels.empty()) return;

    const BatchRenderer::Space space = batch.space();
    batch.setSpace(BatchRenderer::Space::World);
    batch.pushMatrix();
    batch.loadIdentity();
    batch.setTexture(batch.whiteTexture());
    batch.setDepthTest(false);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setCullFace(false);

    ct::Vector<Vec3> lines;
    for (std::size_t i = 0; i < shapes.size(); ++i)
    {
        CollisionShape3D* node = shapes[i];
        if (!node->isVisibleInTree() || node->getKind() == CollisionShapeKind::None) continue;
        const bool selected =
                selection &&
                (selection->isSelected(node) ||
                        (node->getParent() && selection->isSelected(node->getParent())));
        if (node->getKind() == CollisionShapeKind::ConcaveMesh && !selected) continue;
        shape(*node, mSettings.maxConcaveEdges, lines);
        if (lines.empty()) continue;
        const Mat4 global = node->getGlobalTransform();
        const Vec3 x = global[0].xyz().Normalized();
        const Vec3 y = global[1].xyz().Normalized();
        const Vec3 z = global[2].xyz().Normalized();
        const Vec3 position = global.GetTranslation();
        const Vec3 color = gizmoColor(*node);
        float opacity = selected ? 1.0f : mSettings.unselectedOpacity;
        if (node->isDisabled()) opacity *= mSettings.disabledOpacity;
        batch.setColor(color.x, color.y, color.z, opacity);
        batch.begin(BatchRenderer::ModeLines);
        for (std::size_t p = 0; p < lines.size(); ++p)
        {
            const Vec3& q = lines[p];
            const Vec3 world = position + x * q.x + y * q.y + z * q.z;
            batch.vertex3(world.x, world.y, world.z);
        }
        batch.end();
        mLines += static_cast<std::uint32_t>(lines.size() / 2);
        ++mShapes;
    }

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        Joint3D* joint = joints[i];
        if (!joint->isVisibleInTree()) continue;
        const bool selected = selection && selection->isSelected(joint);
        drawJoint(batch, joint, selected ? 1.0f : mSettings.unselectedOpacity);
        ++mJoints;
    }
    for (std::size_t i = 0; i < wheels.size(); ++i)
    {
        VehicleWheel3D* wheel = wheels[i];
        if (!wheel->isVisibleInTree()) continue;
        const bool selected =
                selection &&
                (selection->isSelected(wheel) ||
                        (wheel->getParent() && selection->isSelected(wheel->getParent())));
        drawWheel(batch, wheel, selected ? 1.0f : mSettings.unselectedOpacity);
        ++mWheels;
    }

    batch.popMatrix();
    batch.setSpace(space);
}

std::uint32_t EditorShapeGizmo::shapeCount() const
{
    return mShapes;
}

std::uint32_t EditorShapeGizmo::jointCount() const
{
    return mJoints;
}

std::uint32_t EditorShapeGizmo::wheelCount() const
{
    return mWheels;
}

std::uint32_t EditorShapeGizmo::lineCount() const
{
    return mLines;
}

} // namespace prisma3d
