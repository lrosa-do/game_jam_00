#pragma once

#include "prisma3d/render/MeshStore.h"

inline void addBoxFace(prisma3d::SurfaceData& data, const Math::Vec3& a, const Math::Vec3& b,
        const Math::Vec3& c, const Math::Vec3& d, const Math::Vec3& normal, const Math::Vec4& color)
{
    const std::uint32_t base = static_cast<std::uint32_t>(data.positions.size());
    data.positions.push_back(a);
    data.positions.push_back(b);
    data.positions.push_back(c);
    data.positions.push_back(d);
    for (int i = 0; i < 4; ++i)
    {
        data.normals.push_back(normal);
        data.colors.push_back(color);
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(base + index);
}

inline prisma3d::SurfaceData colouredBox()
{
    prisma3d::SurfaceData data;
    const float h = 0.5f;
    addBoxFace(data, Math::Vec3(-h, 0, h), Math::Vec3(h, 0, h), Math::Vec3(h, 1, h),
            Math::Vec3(-h, 1, h), Math::Vec3(0, 0, 1), Math::Vec4(1, 0, 0, 1));
    addBoxFace(data, Math::Vec3(h, 0, h), Math::Vec3(h, 0, -h), Math::Vec3(h, 1, -h),
            Math::Vec3(h, 1, h), Math::Vec3(1, 0, 0), Math::Vec4(0, 1, 0, 1));
    addBoxFace(data, Math::Vec3(h, 0, -h), Math::Vec3(-h, 0, -h), Math::Vec3(-h, 1, -h),
            Math::Vec3(h, 1, -h), Math::Vec3(0, 0, -1), Math::Vec4(0, 0, 1, 1));
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(-h, 0, h), Math::Vec3(-h, 1, h),
            Math::Vec3(-h, 1, -h), Math::Vec3(-1, 0, 0), Math::Vec4(1, 1, 0, 1));
    addBoxFace(data, Math::Vec3(-h, 1, h), Math::Vec3(h, 1, h), Math::Vec3(h, 1, -h),
            Math::Vec3(-h, 1, -h), Math::Vec3(0, 1, 0), Math::Vec4(1, 1, 1, 1));
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(h, 0, -h), Math::Vec3(h, 0, h),
            Math::Vec3(-h, 0, h), Math::Vec3(0, -1, 0), Math::Vec4(0, 0, 0, 1));
    return data;
}
