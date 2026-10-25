#pragma once

#include "ColouredBox.h"

inline prisma3d::SurfaceData markedBox()
{
    prisma3d::SurfaceData data;
    const float h = 0.5f;
    const Math::Vec3 front(0, 0, 1);
    addBoxFace(data, Math::Vec3(-h, 0, h), Math::Vec3(0, 0, h), Math::Vec3(0, 0.5f, h),
            Math::Vec3(-h, 0.5f, h), front, Math::Vec4(1, 0, 0, 1));
    addBoxFace(data, Math::Vec3(0, 0, h), Math::Vec3(h, 0, h), Math::Vec3(h, 0.5f, h),
            Math::Vec3(0, 0.5f, h), front, Math::Vec4(1, 0, 1, 1));
    addBoxFace(data, Math::Vec3(-h, 0.5f, h), Math::Vec3(0, 0.5f, h), Math::Vec3(0, 1, h),
            Math::Vec3(-h, 1, h), front, Math::Vec4(0, 1, 1, 1));
    addBoxFace(data, Math::Vec3(0, 0.5f, h), Math::Vec3(h, 0.5f, h), Math::Vec3(h, 1, h),
            Math::Vec3(0, 1, h), front, Math::Vec4(1, 1, 0.5f, 1));
    addBoxFace(data, Math::Vec3(h, 0, h), Math::Vec3(h, 0, -h), Math::Vec3(h, 1, -h),
            Math::Vec3(h, 1, h), Math::Vec3(1, 0, 0), Math::Vec4(0, 1, 0, 1));
    addBoxFace(data, Math::Vec3(h, 0, -h), Math::Vec3(-h, 0, -h), Math::Vec3(-h, 1, -h),
            Math::Vec3(h, 1, -h), Math::Vec3(0, 0, -1), Math::Vec4(0, 0, 1, 1));
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(-h, 0, h), Math::Vec3(-h, 1, h),
            Math::Vec3(-h, 1, -h), Math::Vec3(-1, 0, 0), Math::Vec4(1, 1, 0, 1));
    return data;
}
