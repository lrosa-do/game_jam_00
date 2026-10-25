#include "prisma3d/editor/EditorNodeIcons.h"

#include "prisma3d/editor/EditorLightGizmo.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"

#include "platform.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

using Math::Mat4;
using Math::Vec2;
using Math::Vec3;
using Math::Vec4;

namespace
{

ct::String joined(const ct::String& directory, const char* name)
{
    ct::String path = directory;
    if (!path.empty() && path.back() != '/' && path.back() != '\\') path += "/";
    path += name;
    return path;
}

bool isDirectory(const ct::String& path)
{
    PathInfo info;
    return !path.empty() && fs_get_path_info(path.c_str(), &info) &&
           info.type == PATH_TYPE_DIRECTORY;
}

bool nextToken(const char*& cursor, char* out, std::size_t capacity)
{
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r') ++cursor;
    if (*cursor == '\0' || *cursor == '\n') return false;
    std::size_t length = 0;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\r' &&
            *cursor != '\n')
    {
        if (length + 1 >= capacity) return false;
        out[length++] = *cursor++;
    }
    out[length] = '\0';
    return length > 0;
}

float closestOnSegment(const Vec2& point, const Vec2& a, const Vec2& b)
{
    const Vec2 edge = b - a;
    const float lengthSquared = edge.x * edge.x + edge.y * edge.y;
    float t = 0.0f;
    if (lengthSquared > 1.0e-12f)
    {
        t = ((point.x - a.x) * edge.x + (point.y - a.y) * edge.y) / lengthSquared;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    const float dx = point.x - (a.x + edge.x * t);
    const float dy = point.y - (a.y + edge.y * t);
    return sqrtf(dx * dx + dy * dy);
}

bool clipToNear(const EditorView& view, Vec3& a, Vec3& b)
{
    if (view.orthogonal()) return true;
    const Vec3 eye = view.position();
    const Vec3 forward = view.forward();
    const float limit = view.projection.zNear * 2.0f;
    const float da = forward.Dot(a - eye);
    const float db = forward.Dot(b - eye);
    if (da < limit && db < limit) return false;
    if (da < limit) a = a + (b - a) * ((limit - da) / (db - da));
    else if (db < limit)
        b = a + (b - a) * ((limit - da) / (db - da));
    return true;
}

} // namespace

EditorNodeIcons::EditorNodeIcons()
        : mTried(false),
          mIcons(0),
          mMarkers(0)
{
}

EditorNodeIcons::Settings& EditorNodeIcons::settings()
{
    return mSettings;
}

const EditorNodeIcons::Settings& EditorNodeIcons::settings() const
{
    return mSettings;
}

ct::String EditorNodeIcons::defaultDirectory()
{
    const char* app = dir_app();
    if (app && *app)
    {
        const ct::String beside = joined(ct::String(app), "icons");
        if (isDirectory(beside)) return beside;
    }
#ifdef PRISMA3D_EDITOR_ICONS_SOURCE_DIR
    return ct::String(PRISMA3D_EDITOR_ICONS_SOURCE_DIR);
#else
    return ct::String();
#endif
}

bool EditorNodeIcons::loadDefault()
{
    return load(defaultDirectory().c_str());
}

bool EditorNodeIcons::load(const char* directory)
{
    mTried = true;
    TextureStore* store = TextureStore::singleton();
    if (!store || !directory || !*directory) return false;
    const ct::String folder(directory);

    char* listing = file_read_text(joined(folder, "atlas.txt").c_str());
    if (!listing) return false;
    const TextureId texture = store->createFromFile(joined(folder, "atlas.png"), false, false);
    const TextureInfo* info = store->get(texture);
    if (!info || info->width == 0 || info->height == 0)
    {
        free(listing);
        if (texture.valid()) store->destroy(texture);
        return false;
    }

    const float width = static_cast<float>(info->width);
    const float height = static_cast<float>(info->height);
    ct::HashMap<ct::String, Vec4> uv;
    const char* cursor = listing;
    while (*cursor)
    {
        char name[96];
        char numbers[4][24];
        bool good = nextToken(cursor, name, sizeof name);
        for (int i = 0; i < 4 && good; ++i) good = nextToken(cursor, numbers[i], sizeof numbers[i]);
        while (*cursor && *cursor != '\n') ++cursor;
        if (*cursor == '\n') ++cursor;
        if (!good) continue;
        const float x = static_cast<float>(atoi(numbers[0]));
        const float y = static_cast<float>(atoi(numbers[1]));
        const float w = static_cast<float>(atoi(numbers[2]));
        const float h = static_cast<float>(atoi(numbers[3]));
        if (w <= 0.0f || h <= 0.0f || x < 0.0f || y < 0.0f || x + w > width || y + h > height)
            continue;
        const float insetX = 0.5f / width;
        const float insetY = 0.5f / height;
        uv.put(ct::String(name), Vec4(x / width + insetX, y / height + insetY,
                                         (x + w) / width - insetX, (y + h) / height - insetY));
    }
    free(listing);

    if (mTexture.valid()) store->destroy(mTexture);
    mTexture = texture;
    mUv = uv;
    return true;
}

bool EditorNodeIcons::loaded() const
{
    return mTexture.valid() && TextureStore::singleton() != nullptr &&
           TextureStore::singleton()->valid(mTexture);
}

bool EditorNodeIcons::hasIcon(const char* name) const
{
    return name && mUv.find(ct::String(name)) != nullptr;
}

const char* EditorNodeIcons::iconName(const Node3D& node)
{
    if (node.castTo<DirectionalLight3D>()) return "GizmoDirectionalLight";
    if (node.castTo<SpotLight3D>()) return "GizmoSpotLight";
    if (node.castTo<Light3D>()) return "GizmoLight";
    if (node.castTo<Camera3D>()) return "GizmoCamera3D";
    if (node.castTo<AudioListener3D>()) return "GizmoAudioListener3D";
    if (node.castTo<AudioStreamPlayer3D>()) return "Gizmo3DSamplePlayer";
    if (node.castTo<ReflectionProbe3D>()) return "GizmoReflectionProbe";
    if (node.castTo<Decal3D>()) return "GizmoDecal";
    if (node.castTo<ParticleSystem3D>()) return "GizmoGPUParticles3D";
    return nullptr;
}

EditorNodeIcons::Kind EditorNodeIcons::kindOf(const Node3D& node)
{
    if (iconName(node)) return Kind::Icon;
    if (strcmp(node.getClassName(), "Node3D") == 0) return Kind::Marker;
    return Kind::None;
}

Vec3 EditorNodeIcons::tint(const Node3D& node)
{
    if (const Light3D* light = node.castTo<Light3D>()) return EditorLightGizmo::gizmoColor(*light);
    return Vec3(1.0f, 1.0f, 1.0f);
}

float EditorNodeIcons::billboardScale(const EditorView& view, const Vec3& position)
{
    if (view.orthogonal()) return view.projection.size * 0.5f;
    return view.forward().Dot(position - view.position());
}

bool EditorNodeIcons::iconHit(const EditorView& view, const Vec3& position, float iconScale,
        float x, float y)
{
    if (!view.orthogonal() && view.behind(position)) return false;
    const float half = iconScale * billboardScale(view, position);
    const Vec3 corner = (view.right() + view.up()) * half;
    const Vec2 center = view.project(position);
    const Vec2 p0 = view.project(position + corner);
    const Vec2 p1 = view.project(position - corner);
    const float halfWidth = fabsf(p1.x - p0.x) * 0.5f;
    const float halfHeight = fabsf(p1.y - p0.y) * 0.5f;
    return x >= center.x - halfWidth && x <= center.x + halfWidth && y >= center.y - halfHeight &&
           y <= center.y + halfHeight;
}

void EditorNodeIcons::markerSegments(const Mat4& global, float extents, ct::Vector<Vec3>& points)
{
    points.clear();
    for (int axis = 0; axis < 3; ++axis)
    {
        Vec3 direction(0.0f, 0.0f, 0.0f);
        if (axis == 0) direction.x = extents;
        else if (axis == 1)
            direction.y = extents;
        else
            direction.z = extents;
        points.push_back(global.TransformPoint(direction));
        points.push_back(global.TransformPoint(Vec3(0.0f, 0.0f, 0.0f)));
        points.push_back(global.TransformPoint(Vec3(0.0f, 0.0f, 0.0f)));
        points.push_back(global.TransformPoint(-direction));
    }
}

bool EditorNodeIcons::markerHit(const EditorView& view, const Mat4& global, float extents,
        float radius, float x, float y)
{
    ct::Vector<Vec3> points;
    markerSegments(global, extents, points);
    const Vec2 mouse(x, y);
    for (std::size_t i = 0; i + 1 < points.size(); i += 2)
    {
        Vec3 a = points[i];
        Vec3 b = points[i + 1];
        if (!clipToNear(view, a, b)) continue;
        if (closestOnSegment(mouse, view.project(a), view.project(b)) < radius) return true;
    }
    return false;
}

float EditorNodeIcons::pickDistance(const EditorView& view, const Node3D& node,
        const Settings& settings, float x, float y)
{
    const Kind kind = kindOf(node);
    if (kind == Kind::None) return -1.0f;
    const Mat4 global = node.getGlobalTransform();
    const Vec3 position = global.GetTranslation();
    bool hit = false;
    if (kind == Kind::Icon) hit = iconHit(view, position, settings.iconScale, x, y);
    else
        hit = markerHit(view, global, settings.markerExtents, settings.pickRadius, x, y);
    if (!hit) return -1.0f;
    const float distance = (position - view.rayOrigin(x, y)).Dot(view.rayDirection(x, y));
    return distance > 0.0f ? distance : 0.0f;
}

void EditorNodeIcons::collect(Node* node, ct::Vector<Node3D*>& out) const
{
    if (!node) return;
    if (Node3D* spatial = node->castTo<Node3D>())
        if (kindOf(*spatial) != Kind::None && spatial->isVisibleInTree()) out.push_back(spatial);
    for (std::size_t i = 0; i < node->getChildCount(); ++i) collect(node->getChild(i), out);
}

void EditorNodeIcons::draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
        const EditorView& view)
{
    mIcons = 0;
    mMarkers = 0;
    if (!mTried) loadDefault();

    ct::Vector<Node3D*> nodes;
    collect(root, nodes);
    if (nodes.empty()) return;

    const BatchRenderer::Space space = batch.space();
    batch.setSpace(BatchRenderer::Space::World);
    batch.pushMatrix();
    batch.loadIdentity();
    batch.setDepthTest(false);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setCullFace(false);

    TextureStore* store = TextureStore::singleton();
    const bool textured = loaded() && store;
    const Vec3 right = view.right();
    const Vec3 up = view.up();

    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        Node3D* node = nodes[i];
        const Mat4 global = node->getGlobalTransform();
        if (kindOf(*node) == Kind::Marker)
        {
            ct::Vector<Vec3> points;
            markerSegments(global, mSettings.markerExtents, points);
            batch.setTexture(batch.whiteTexture());
            for (std::size_t k = 0; k + 3 < points.size(); k += 4)
            {
                const Vec3 color = mSettings.axisColor[k / 4];
                const Vec3 dark = color * (1.0f - mSettings.markerDarken);
                batch.setColor(color.x, color.y, color.z, 1.0f);
                batch.begin(BatchRenderer::ModeLines);
                batch.vertex3(points[k].x, points[k].y, points[k].z);
                batch.vertex3(points[k + 1].x, points[k + 1].y, points[k + 1].z);
                batch.end();
                batch.setColor(dark.x, dark.y, dark.z, 1.0f);
                batch.begin(BatchRenderer::ModeLines);
                batch.vertex3(points[k + 2].x, points[k + 2].y, points[k + 2].z);
                batch.vertex3(points[k + 3].x, points[k + 3].y, points[k + 3].z);
                batch.end();
            }
            ++mMarkers;
            continue;
        }

        if (!textured) continue;
        const char* name = iconName(*node);
        const Vec4* uv = name ? mUv.find(ct::String(name)) : nullptr;
        if (!uv) continue;
        const Vec3 position = global.GetTranslation();
        if (!view.orthogonal() && view.behind(position)) continue;

        const float half = mSettings.iconScale * billboardScale(view, position);
        Vec3 color = tint(*node);
        const bool selected = selection && selection->isSelected(node);
        if (!selected) color = color * mSettings.unselectedShade;

        const Vec3 corner[4] = { position + (up - right) * half, position + (up + right) * half,
            position + (right - up) * half, position - (up + right) * half };
        const Vec2 coordinate[4] = { Vec2(uv->x, uv->y), Vec2(uv->z, uv->y), Vec2(uv->z, uv->w),
            Vec2(uv->x, uv->w) };
        batch.setTexture(store->gpu(mTexture));
        batch.setColor(color.x, color.y, color.z, 1.0f);
        batch.begin(BatchRenderer::ModeTriangles);
        const int order[6] = { 0, 1, 2, 0, 2, 3 };
        for (int k = 0; k < 6; ++k)
        {
            batch.setTexcoord(coordinate[order[k]].x, coordinate[order[k]].y);
            batch.vertex3(corner[order[k]].x, corner[order[k]].y, corner[order[k]].z);
        }
        batch.end();
        ++mIcons;
    }

    batch.setTexture(batch.whiteTexture());
    batch.popMatrix();
    batch.setSpace(space);
}

std::uint32_t EditorNodeIcons::iconCount() const
{
    return mIcons;
}

std::uint32_t EditorNodeIcons::markerCount() const
{
    return mMarkers;
}

} // namespace prisma3d
