#include "prisma3d/editor/EditorLightGizmo.h"

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"

#include <math.h>

namespace prisma3d
{

using Math::Mat4;
using Math::Vec3;

namespace
{

const float kPi = 3.14159265358979f;
const float kDegreesToRadians = kPi / 180.0f;
const int kCircleSegments = 120;
const float kArrowLength = 1.5f;

void addLine(ct::Vector<Vec3>& lines, const Vec3& a, const Vec3& b)
{
    lines.push_back(a);
    lines.push_back(b);
}

} // namespace

EditorLightGizmo::EditorLightGizmo()
        : mLights(0),
          mLines(0)
{
}

EditorLightGizmo::Settings& EditorLightGizmo::settings()
{
    return mSettings;
}

const EditorLightGizmo::Settings& EditorLightGizmo::settings() const
{
    return mSettings;
}

Vec3 EditorLightGizmo::gizmoColor(const Light3D& light)
{
    const Vec3 color = light.getColor();
    float peak = color.x > color.y ? color.x : color.y;
    if (color.z > peak) peak = color.z;
    if (peak <= 1.0e-6f) return Vec3(1.0f, 1.0f, 1.0f);
    return Vec3(color.x / peak, color.y / peak, color.z / peak);
}

void EditorLightGizmo::shape(const Light3D& light, Shape& out)
{
    out.primary.clear();
    out.secondary.clear();
    out.billboard.clear();

    if (light.castTo<DirectionalLight3D>())
    {
        const Vec3 arrow[7] = { Vec3(0.0f, 0.0f, -1.0f), Vec3(0.0f, 0.8f, 0.0f),
            Vec3(0.0f, 0.3f, 0.0f), Vec3(0.0f, 0.3f, kArrowLength), Vec3(0.0f, -0.3f, kArrowLength),
            Vec3(0.0f, -0.3f, 0.0f), Vec3(0.0f, -0.8f, 0.0f) };
        for (int side = 0; side < 2; ++side)
        {
            for (int j = 0; j < 7; ++j)
            {
                Vec3 v1 = arrow[j] - Vec3(0.0f, 0.0f, kArrowLength);
                Vec3 v2 = arrow[(j + 1) % 7] - Vec3(0.0f, 0.0f, kArrowLength);
                if (side == 1)
                {
                    v1 = Vec3(-v1.y, v1.x, v1.z);
                    v2 = Vec3(-v2.y, v2.x, v2.z);
                }
                addLine(out.primary, v1, v2);
            }
        }
    }
    else if (const OmniLight3D* omni = light.castTo<OmniLight3D>())
    {
        const float r = omni->getRange();
        for (int i = 0; i < kCircleSegments; ++i)
        {
            const float ra = static_cast<float>(i * 3) * kDegreesToRadians;
            const float rb = static_cast<float>((i + 1) * 3) * kDegreesToRadians;
            const float ax = sinf(ra) * r;
            const float ay = cosf(ra) * r;
            const float bx = sinf(rb) * r;
            const float by = cosf(rb) * r;
            addLine(out.secondary, Vec3(ax, 0.0f, ay), Vec3(bx, 0.0f, by));
            addLine(out.secondary, Vec3(0.0f, ax, ay), Vec3(0.0f, bx, by));
            addLine(out.secondary, Vec3(ax, ay, 0.0f), Vec3(bx, by, 0.0f));
            addLine(out.billboard, Vec3(ax, ay, 0.0f), Vec3(bx, by, 0.0f));
        }
    }
    else if (const SpotLight3D* spot = light.castTo<SpotLight3D>())
    {
        const float r = spot->getRange();
        const float angle = spot->getSpotAngle() * kDegreesToRadians;
        const float w = r * sinf(angle);
        const float d = r * cosf(angle);
        for (int i = 0; i < kCircleSegments; ++i)
        {
            const float ra = static_cast<float>(i * 3) * kDegreesToRadians;
            const float rb = static_cast<float>((i + 1) * 3) * kDegreesToRadians;
            const float ax = sinf(ra) * w;
            const float ay = cosf(ra) * w;
            const float bx = sinf(rb) * w;
            const float by = cosf(rb) * w;
            addLine(out.primary, Vec3(ax, ay, -d), Vec3(bx, by, -d));
            if (i % 15 == 0) addLine(out.secondary, Vec3(ax, ay, -d), Vec3(0.0f, 0.0f, 0.0f));
        }
        addLine(out.primary, Vec3(0.0f, 0.0f, -r), Vec3(0.0f, 0.0f, 0.0f));
    }
}

void EditorLightGizmo::collect(Node* node, ct::Vector<Light3D*>& out) const
{
    if (!node) return;
    if (Light3D* light = node->castTo<Light3D>()) out.push_back(light);
    for (std::size_t i = 0; i < node->getChildCount(); ++i) collect(node->getChild(i), out);
}

void EditorLightGizmo::draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
        const EditorView& view)
{
    mLights = 0;
    mLines = 0;
    ct::Vector<Light3D*> lights;
    collect(root, lights);
    if (lights.empty()) return;

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

    const Vec3 viewUp = view.up();
    const Vec3 viewRight = view.right();

    Shape shapeLines;
    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        Light3D* light = lights[i];
        if (!light->isVisibleInTree()) continue;
        shape(*light, shapeLines);
        const Mat4 global = light->getGlobalTransform();
        const Vec3 x = global[0].xyz().Normalized();
        const Vec3 y = global[1].xyz().Normalized();
        const Vec3 z = global[2].xyz().Normalized();
        const Vec3 position = global.GetTranslation();
        const Vec3 color = gizmoColor(*light);
        const bool selected = selection && selection->isSelected(light);
        const float opacity = selected ? 1.0f : mSettings.unselectedOpacity;

        const ct::Vector<Vec3>* sets[3] = { &shapeLines.primary, &shapeLines.secondary,
            &shapeLines.billboard };
        const float alphas[3] = { opacity, opacity * mSettings.secondaryOpacity,
            opacity * mSettings.secondaryOpacity };
        for (int k = 0; k < 3; ++k)
        {
            const ct::Vector<Vec3>& points = *sets[k];
            if (points.empty()) continue;
            batch.setColor(color.x, color.y, color.z, alphas[k]);
            batch.begin(BatchRenderer::ModeLines);
            for (std::size_t p = 0; p < points.size(); ++p)
            {
                const Vec3& q = points[p];
                const Vec3 world = k == 2 ? position + viewRight * q.x + viewUp * q.y
                                          : position + x * q.x + y * q.y + z * q.z;
                batch.vertex3(world.x, world.y, world.z);
            }
            batch.end();
            mLines += static_cast<std::uint32_t>(points.size() / 2);
        }
        ++mLights;
    }

    batch.popMatrix();
    batch.setSpace(space);
}

std::uint32_t EditorLightGizmo::lightCount() const
{
    return mLights;
}

std::uint32_t EditorLightGizmo::lineCount() const
{
    return mLines;
}

} // namespace prisma3d
