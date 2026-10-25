#include "prisma3d/editor/EditorCameraGizmo.h"

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node.h"

#include <math.h>

namespace prisma3d
{

using Math::Mat4;
using Math::Vec3;
using Math::Vec4;

namespace
{

const float kPi = 3.14159265358979f;
const float kDegreesToRadians = kPi / 180.0f;

void addTriangle(ct::Vector<Vec3>& lines, const Vec3& a, const Vec3& b, const Vec3& c)
{
    lines.push_back(a);
    lines.push_back(b);
    lines.push_back(b);
    lines.push_back(c);
    lines.push_back(c);
    lines.push_back(a);
}

void addQuad(ct::Vector<Vec3>& lines, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    lines.push_back(a);
    lines.push_back(b);
    lines.push_back(b);
    lines.push_back(c);
    lines.push_back(c);
    lines.push_back(d);
    lines.push_back(d);
    lines.push_back(a);
}

} // namespace

EditorCameraGizmo::EditorCameraGizmo()
        : mCameras(0),
          mLines(0)
{
}

EditorCameraGizmo::Settings& EditorCameraGizmo::settings()
{
    return mSettings;
}

const EditorCameraGizmo::Settings& EditorCameraGizmo::settings() const
{
    return mSettings;
}

void EditorCameraGizmo::frustumLines(const Camera3D& camera, float aspect, ct::Vector<Vec3>& lines)
{
    lines.clear();
    const float viewportAspect = aspect > 0.0f ? aspect : 1.0f;
    const float factorX = viewportAspect > 1.0f ? 1.0f : viewportAspect;
    const float factorY = viewportAspect > 1.0f ? 1.0f / viewportAspect : 1.0f;

    if (camera.getProjection() == CameraProjection::Perspective)
    {
        const float fov = camera.getFov() * 0.5f;
        const float hsize = sinf(fov * kDegreesToRadians);
        const float depth = -cosf(fov * kDegreesToRadians);

        Vec3 side(hsize * factorX, 0.0f, depth * factorY);
        Vec3 nside(-side.x, side.y, side.z);
        const Vec3 up(0.0f, hsize * factorY, 0.0f);
        const Vec3 origin(0.0f, 0.0f, 0.0f);

        addTriangle(lines, origin, side + up, side - up);
        addTriangle(lines, origin, nside + up, nside - up);
        addTriangle(lines, origin, side + up, nside + up);
        addTriangle(lines, origin, side - up, nside - up);

        side.x = side.x < hsize * 0.25f ? side.x : hsize * 0.25f;
        nside.x = -side.x;
        const Vec3 tup(0.0f, up.y + hsize * 0.5f, side.z);
        addTriangle(lines, tup, side + up, nside + up);
    }
    else
    {
        const float keep = camera.getSize() * 0.5f;
        Vec3 right(keep * viewportAspect, 0.0f, 0.0f);
        const Vec3 up(0.0f, keep, 0.0f);
        const Vec3 back(0.0f, 0.0f, -1.0f);

        addQuad(lines, -up - right, -up + right, up + right, up - right);
        addQuad(lines, -up - right + back, -up + right + back, up + right + back,
                up - right + back);
        addQuad(lines, up + right, up + right + back, up - right + back, up - right);
        addQuad(lines, -up + right, -up + right + back, -up - right + back, -up - right);

        right.x = right.x < keep * 0.25f ? right.x : keep * 0.25f;
        const Vec3 tup(0.0f, up.y + keep * 0.5f, back.z);
        addTriangle(lines, tup, right + up + back, -right + up + back);
    }
}

void EditorCameraGizmo::collect(Node* node, ct::Vector<Camera3D*>& out) const
{
    if (!node) return;
    if (Camera3D* camera = node->castTo<Camera3D>()) out.push_back(camera);
    for (std::size_t i = 0; i < node->getChildCount(); ++i) collect(node->getChild(i), out);
}

void EditorCameraGizmo::draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
        float aspect)
{
    mCameras = 0;
    mLines = 0;
    ct::Vector<Camera3D*> cameras;
    collect(root, cameras);
    if (cameras.empty()) return;

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
    for (std::size_t i = 0; i < cameras.size(); ++i)
    {
        Camera3D* camera = cameras[i];
        if (!camera->isVisibleInTree()) continue;
        const Mat4 global = camera->getGlobalTransform();
        const Vec3 x = global[0].xyz().Normalized();
        const Vec3 y = global[1].xyz().Normalized();
        const Vec3 z = global[2].xyz().Normalized();
        const Vec3 position = global.GetTranslation();

        frustumLines(*camera, aspect, lines);
        Vec4 color = mSettings.color;
        const bool selected = selection && selection->isSelected(camera);
        if (!selected) color.w *= mSettings.unselectedOpacity;
        batch.setColor(color.x, color.y, color.z, color.w);
        batch.begin(BatchRenderer::ModeLines);
        for (std::size_t k = 0; k < lines.size(); ++k)
        {
            const Vec3& p = lines[k];
            const Vec3 world = position + x * p.x + y * p.y + z * p.z;
            batch.vertex3(world.x, world.y, world.z);
        }
        batch.end();
        ++mCameras;
        mLines += static_cast<std::uint32_t>(lines.size() / 2);
    }

    batch.popMatrix();
    batch.setSpace(space);
}

std::uint32_t EditorCameraGizmo::cameraCount() const
{
    return mCameras;
}

std::uint32_t EditorCameraGizmo::lineCount() const
{
    return mLines;
}

} // namespace prisma3d
