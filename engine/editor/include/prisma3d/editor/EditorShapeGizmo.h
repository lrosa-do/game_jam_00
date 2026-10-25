#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class CollisionShape3D;
class EditorSelection;
struct EditorView;
class Joint3D;
class Node;
class VehicleWheel3D;

class EditorShapeGizmo
{
public:
    struct Settings
    {
        float unselectedOpacity = 0.6f;
        float disabledOpacity = 0.3f;
        std::uint32_t maxConcaveEdges = 60000;
        bool shapes = true;
        bool joints = true;
    };

    EditorShapeGizmo();

    Settings& settings();
    const Settings& settings() const;

    static Math::Vec3 gizmoColor(const CollisionShape3D& shape);
    static Math::Vec3 jointColor();
    static void shape(const CollisionShape3D& shape, std::uint32_t maxConcaveEdges,
            ct::Vector<Math::Vec3>& lines);

    void draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
            const EditorView& view);

    std::uint32_t shapeCount() const;
    std::uint32_t jointCount() const;
    std::uint32_t wheelCount() const;
    std::uint32_t lineCount() const;

private:
    void collect(Node* node, ct::Vector<CollisionShape3D*>& shapes, ct::Vector<Joint3D*>& joints,
            ct::Vector<VehicleWheel3D*>& wheels) const;
    void drawJoint(BatchRenderer& batch, Joint3D* joint, float opacity);
    void drawWheel(BatchRenderer& batch, VehicleWheel3D* wheel, float opacity);

    Settings mSettings;
    std::uint32_t mShapes;
    std::uint32_t mJoints;
    std::uint32_t mWheels;
    std::uint32_t mLines;
};

} // namespace prisma3d
