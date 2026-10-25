#pragma once

#include "prisma3d/editor/EditorCamera.h"

#include "mathc.h"

namespace prisma3d
{

struct EditorView
{
    Math::Mat4 camera = Math::Mat4::Identity();
    EditorCamera::Projection projection;
    float width = 1.0f;
    float height = 1.0f;

    bool orthogonal() const;
    Math::Vec3 position() const;
    Math::Vec3 right() const;
    Math::Vec3 up() const;
    Math::Vec3 forward() const;
    Math::Vec3 rayOrigin(float x, float y) const;
    Math::Vec3 rayDirection(float x, float y) const;
    Math::Vec2 project(const Math::Vec3& world) const;
    bool behind(const Math::Vec3& world) const;
};

} // namespace prisma3d
