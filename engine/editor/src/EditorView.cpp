#include "prisma3d/editor/EditorView.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kDegreesToRadians = 3.14159265358979323846f / 180.0f;

} // namespace

bool EditorView::orthogonal() const
{
    return projection.orthogonal;
}

Math::Vec3 EditorView::position() const
{
    return camera.GetTranslation();
}

Math::Vec3 EditorView::right() const
{
    return camera.col0.xyz().Normalized();
}

Math::Vec3 EditorView::up() const
{
    return camera.col1.xyz().Normalized();
}

Math::Vec3 EditorView::forward() const
{
    return -camera.col2.xyz().Normalized();
}

Math::Vec3 EditorView::rayDirection(float x, float y) const
{
    if (projection.orthogonal) return forward();
    const float aspect = width / height;
    const float halfY = projection.zNear * tanf(projection.fov * kDegreesToRadians * 0.5f);
    const float halfX = halfY * aspect;
    const Math::Vec3 local(((x / width) * 2.0f - 1.0f) * halfX,
            ((1.0f - (y / height)) * 2.0f - 1.0f) * halfY, -projection.zNear);
    return camera.TransformVector(local.Normalized()).Normalized();
}

Math::Vec3 EditorView::rayOrigin(float x, float y) const
{
    if (!projection.orthogonal) return position();
    const float aspect = width / height;
    const float horizontal = projection.size * aspect;
    const float vertical = projection.size;
    const Math::Vec3 local((x / width) * horizontal - horizontal * 0.5f,
            (1.0f - y / height) * vertical - vertical * 0.5f, -projection.zNear);
    return camera.TransformPoint(local);
}

Math::Vec2 EditorView::project(const Math::Vec3& world) const
{
    const Math::Vec3 local = camera.InverseAffine().TransformPoint(world);
    const float aspect = width / height;
    float ndcX = 0.0f;
    float ndcY = 0.0f;
    if (projection.orthogonal)
    {
        ndcX = local.x / (projection.size * aspect * 0.5f);
        ndcY = local.y / (projection.size * 0.5f);
    }
    else
    {
        const float depth = -local.z;
        const float tanHalf = tanf(projection.fov * kDegreesToRadians * 0.5f);
        const float safe = fabsf(depth) < 1e-6f ? 1e-6f : depth;
        ndcX = local.x / (safe * tanHalf * aspect);
        ndcY = local.y / (safe * tanHalf);
    }
    return Math::Vec2((ndcX * 0.5f + 0.5f) * width, (-ndcY * 0.5f + 0.5f) * height);
}

bool EditorView::behind(const Math::Vec3& world) const
{
    return forward().Dot(world - position()) < projection.zNear;
}

} // namespace prisma3d
