#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class EditorSelection;
struct EditorView;
class Light3D;
class Node;

class EditorLightGizmo
{
public:
    struct Settings
    {
        float unselectedOpacity = 0.7f;
        float secondaryOpacity = 0.5f;
    };

    struct Shape
    {
        ct::Vector<Math::Vec3> primary;
        ct::Vector<Math::Vec3> secondary;
        ct::Vector<Math::Vec3> billboard;
    };

    EditorLightGizmo();

    Settings& settings();
    const Settings& settings() const;

    static Math::Vec3 gizmoColor(const Light3D& light);
    static void shape(const Light3D& light, Shape& out);

    void draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
            const EditorView& view);

    std::uint32_t lightCount() const;
    std::uint32_t lineCount() const;

private:
    void collect(Node* node, ct::Vector<Light3D*>& out) const;

    Settings mSettings;
    std::uint32_t mLights;
    std::uint32_t mLines;
};

} // namespace prisma3d
