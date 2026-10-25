#pragma once

#include "Check.h"

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/import/PMeshFormat.h"

#include <string.h>

namespace prisma3d
{

struct MemoryImporter final : ModelImporter
{
    ct::Vector<SurfaceData> surfaces;

    const char* name() const override
    {
        return "memory";
    }
    bool supports(const char*) const override
    {
        return false;
    }
    bool import(const ct::String&, ModelData&) override
    {
        return false;
    }
    bool importMemory(const std::uint8_t*, std::size_t, const ct::String&, ModelData&) override
    {
        return false;
    }
    bool readPrimitive(const ModelData&, const ModelPrimitiveData& primitive,
            SurfaceData& out) const override
    {
        if (primitive.source >= surfaces.size()) return false;
        out = surfaces[primitive.source];
        return true;
    }
    bool readImage(const ModelData&, const ModelImageData&,
            ct::Vector<std::uint8_t>&) const override
    {
        return false;
    }
};

template<typename T>
bool sameBytes(const ct::Vector<T>& a, const ct::Vector<T>& b)
{
    if (a.size() != b.size()) return false;
    return a.empty() || memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}

inline bool sameSurface(const SurfaceData& a, const SurfaceData& b)
{
    return a.topology == b.topology && sameBytes(a.positions, b.positions) &&
           sameBytes(a.normals, b.normals) && sameBytes(a.tangents, b.tangents) &&
           sameBytes(a.colors, b.colors) && sameBytes(a.uvs, b.uvs) && sameBytes(a.uvs2, b.uvs2) &&
           sameBytes(a.joints, b.joints) && sameBytes(a.weights, b.weights) &&
           sameBytes(a.indices, b.indices);
}

inline std::uint32_t maskOf(const SurfaceData& s)
{
    std::uint32_t mask = kVertexPosition;
    if (!s.normals.empty()) mask |= kVertexNormal;
    if (!s.tangents.empty()) mask |= kVertexTangent;
    if (!s.colors.empty()) mask |= kVertexColor;
    if (!s.uvs.empty()) mask |= kVertexUv;
    if (!s.uvs2.empty()) mask |= kVertexUv2;
    if (!s.joints.empty()) mask |= kVertexJoints;
    if (!s.weights.empty()) mask |= kVertexWeights;
    return mask;
}

inline bool sameMat(const Math::Mat4& a, const Math::Mat4& b)
{
    return memcmp(&a, &b, sizeof(Math::Mat4)) == 0;
}

inline bool sameSkeleton(const Skeleton& a, const Skeleton& b)
{
    if (a.boneCount() != b.boneCount() || a.finalized() != b.finalized()) return false;
    for (std::uint32_t i = 0; i < a.boneCount(); ++i)
    {
        if (!(a.bone(i).name == b.bone(i).name) || a.bone(i).parent != b.bone(i).parent ||
                !sameMat(a.bone(i).bindLocal, b.bone(i).bindLocal) ||
                !sameMat(a.bone(i).inverseBind, b.bone(i).inverseBind))
            return false;
    }
    return sameBytes(a.order(), b.order());
}

inline bool sameClip(const AnimationClip& a, const AnimationClip& b)
{
    if (!(a.name() == b.name()) || a.duration() != b.duration() || a.skeleton() != b.skeleton() ||
            a.loopMode() != b.loopMode() || a.tracks().size() != b.tracks().size() ||
            a.nodeTracks().size() != b.nodeTracks().size() ||
            a.methodTracks().size() != b.methodTracks().size() ||
            a.valueTracks().size() != b.valueTracks().size() ||
            a.animationTracks().size() != b.animationTracks().size() ||
            a.events().size() != b.events().size())
        return false;
    for (std::size_t i = 0; i < a.tracks().size(); ++i)
    {
        const BoneTrack& x = a.tracks()[i];
        const BoneTrack& y = b.tracks()[i];
        if (x.bone != y.bone || !sameBytes(x.times, y.times) ||
                !sameBytes(x.positions, y.positions) || !sameBytes(x.rotations, y.rotations) ||
                !sameBytes(x.scales, y.scales))
            return false;
    }
    for (std::size_t i = 0; i < a.nodeTracks().size(); ++i)
    {
        const NodeTrack& x = a.nodeTracks()[i];
        const NodeTrack& y = b.nodeTracks()[i];
        if (!(x.path == y.path) || !sameBytes(x.positionTimes, y.positionTimes) ||
                !sameBytes(x.positions, y.positions) ||
                !sameBytes(x.rotationTimes, y.rotationTimes) ||
                !sameBytes(x.rotations, y.rotations) || !sameBytes(x.scaleTimes, y.scaleTimes) ||
                !sameBytes(x.scales, y.scales))
            return false;
    }
    for (std::size_t i = 0; i < a.methodTracks().size(); ++i)
    {
        const MethodTrack& x = a.methodTracks()[i];
        const MethodTrack& y = b.methodTracks()[i];
        if (!(x.path == y.path) || !sameBytes(x.times, y.times) ||
                x.methods.size() != y.methods.size())
            return false;
        for (std::size_t k = 0; k < x.methods.size(); ++k)
            if (!(x.methods[k] == y.methods[k])) return false;
    }
    for (std::size_t i = 0; i < a.valueTracks().size(); ++i)
    {
        const ValueTrack& x = a.valueTracks()[i];
        const ValueTrack& y = b.valueTracks()[i];
        if (!(x.path == y.path) || !(x.property == y.property) || !sameBytes(x.times, y.times) ||
                !sameBytes(x.values, y.values))
            return false;
    }
    for (std::size_t i = 0; i < a.animationTracks().size(); ++i)
    {
        const AnimationTrack& x = a.animationTracks()[i];
        const AnimationTrack& y = b.animationTracks()[i];
        if (!(x.path == y.path) || !sameBytes(x.times, y.times) ||
                x.animations.size() != y.animations.size())
            return false;
        for (std::size_t k = 0; k < x.animations.size(); ++k)
            if (!(x.animations[k] == y.animations[k])) return false;
    }
    for (std::size_t i = 0; i < a.events().size(); ++i)
        if (a.events()[i].time != b.events()[i].time || !(a.events()[i].name == b.events()[i].name))
            return false;
    return true;
}

inline int modelDifferences(const ModelData& a, const ModelImporter& importerA, const ModelData& b,
        const ModelImporter& importerB, bool checkMask = true)
{
    int differences = 0;
    auto diff = [&](bool same) { differences += same ? 0 : 1; };

    diff(a.nodes.size() == b.nodes.size());
    diff(a.roots.size() == b.roots.size());
    diff(a.meshes.size() == b.meshes.size());
    diff(a.materials.size() == b.materials.size());
    diff(a.cameras.size() == b.cameras.size());
    diff(a.skeletons.size() == b.skeletons.size());
    diff(a.animations.size() == b.animations.size());
    if (differences != 0) return differences;

    for (std::size_t i = 0; i < a.nodes.size(); ++i)
    {
        const ModelNodeData& x = a.nodes[i];
        const ModelNodeData& y = b.nodes[i];
        diff(x.name == y.name);
        diff(x.parent == y.parent && x.mesh == y.mesh && x.camera == y.camera &&
                x.skeleton == y.skeleton);
        diff(memcmp(&x.translation, &y.translation, sizeof(x.translation)) == 0);
        diff(memcmp(&x.rotation, &y.rotation, sizeof(x.rotation)) == 0);
        diff(memcmp(&x.scale, &y.scale, sizeof(x.scale)) == 0);
        diff(sameBytes(x.children, y.children));
    }
    diff(sameBytes(a.roots, b.roots));
    for (std::size_t i = 0; i < a.cameras.size(); ++i)
    {
        const ModelCameraData& x = a.cameras[i];
        const ModelCameraData& y = b.cameras[i];
        diff(x.perspective == y.perspective && x.yfov == y.yfov && x.aspect == y.aspect &&
                x.zNear == y.zNear && x.zFar == y.zFar && x.xmag == y.xmag && x.ymag == y.ymag);
    }
    for (std::size_t i = 0; i < a.materials.size(); ++i)
        diff(a.materials[i].name == b.materials[i].name);

    for (std::size_t m = 0; m < a.meshes.size(); ++m)
    {
        const ModelMeshData& x = a.meshes[m];
        const ModelMeshData& y = b.meshes[m];
        diff(x.name == y.name && x.primitives.size() == y.primitives.size());
        if (x.primitives.size() != y.primitives.size()) continue;
        diff(x.shadowSurfaces.size() == y.shadowSurfaces.size());
        if (x.shadowSurfaces.size() == y.shadowSurfaces.size())
            for (std::size_t s = 0; s < x.shadowSurfaces.size(); ++s)
                diff(sameSurface(x.shadowSurfaces[s], y.shadowSurfaces[s]));
        for (std::size_t p = 0; p < x.primitives.size(); ++p)
        {
            const ModelPrimitiveData& px = x.primitives[p];
            const ModelPrimitiveData& py = y.primitives[p];
            SurfaceData sx;
            SurfaceData sy;
            const bool readX = importerA.readPrimitive(a, px, sx);
            const bool readY = importerB.readPrimitive(b, py, sy);
            diff(readX && readY);
            if (!readX || !readY) continue;
            diff(sameSurface(sx, sy));
            if (checkMask) diff(py.attributeMask == maskOf(sx));
            diff(px.topology == py.topology && px.material == py.material &&
                    px.vertexCount == py.vertexCount && px.skinned == py.skinned &&
                    px.hasAabb == py.hasAabb);
            diff(py.vertexCount == sx.positions.size() && py.indexCount == sx.indices.size());
            if (px.hasAabb) diff(memcmp(&px.aabb, &py.aabb, sizeof(px.aabb)) == 0);
        }
    }
    for (std::size_t i = 0; i < a.skeletons.size(); ++i)
        diff(sameSkeleton(a.skeletons[i], b.skeletons[i]));
    for (std::size_t i = 0; i < a.animations.size(); ++i)
        diff(sameClip(a.animations[i], b.animations[i]));
    return differences;
}

} // namespace prisma3d
