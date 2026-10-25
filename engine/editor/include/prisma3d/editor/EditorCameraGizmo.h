#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class Camera3D;
class EditorSelection;
class Node;

class EditorCameraGizmo
{
public:
    struct Settings
    {
        Math::Vec4 color = Math::Vec4(0.8f, 0.4f, 0.8f, 1.0f);
        float unselectedOpacity = 0.7f;
    };

    EditorCameraGizmo();

    Settings& settings();
    const Settings& settings() const;

    static void frustumLines(const Camera3D& camera, float aspect, ct::Vector<Math::Vec3>& lines);

    void draw(BatchRenderer& batch, Node* root, const EditorSelection* selection, float aspect);

    std::uint32_t cameraCount() const;
    std::uint32_t lineCount() const;

private:
    void collect(Node* node, ct::Vector<Camera3D*>& out) const;

    Settings mSettings;
    std::uint32_t mCameras;
    std::uint32_t mLines;
};

} // namespace prisma3d
