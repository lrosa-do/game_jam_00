#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/core/FileSystem.h"

#include "prisma3d/import/PMeshFormat.h"
#include "prisma3d/import/PMeshIO.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

using namespace pmesh;

static_assert(sizeof(Math::Vec2) == 8, "Vec2 must be packed");
static_assert(sizeof(Math::Vec3) == 12, "Vec3 must be packed");
static_assert(sizeof(Math::Vec4) == 16, "Vec4 must be packed");
static_assert(sizeof(Math::Quaternion) == 16, "Quaternion must be packed");

const std::uint32_t kMaxTopology = static_cast<std::uint32_t>(prisma::Topology::Patches);

bool bad(ct::String& error, const ct::String& message)
{
    error = message;
    return false;
}

ct::String numbered(const char* prefix, std::size_t index, const char* suffix)
{
    char text[48];
    snprintf(text, sizeof(text), "%s%zu%s", prefix, index, suffix);
    return ct::String(text);
}

bool validBox(const Math::Box& box)
{
    const float values[6] = { box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z };
    for (int i = 0; i < 6; ++i)
        if (values[i] - values[i] != 0.0f) return false;
    return box.min.x <= box.max.x && box.min.y <= box.max.y && box.min.z <= box.max.z;
}

bool indicesFit16(const ct::Vector<std::uint32_t>& indices)
{
    for (std::size_t i = 0; i < indices.size(); ++i)
        if (indices[i] > 0xFFFFu) return false;
    return true;
}

bool checkIndices(const ct::Vector<std::uint32_t>& indices, std::size_t vertexCount)
{
    if (indices.size() > kMaxIndices) return false;
    for (std::size_t i = 0; i < indices.size(); ++i)
        if (indices[i] >= vertexCount) return false;
    return true;
}

void writeIndices(Writer& w, const ct::Vector<std::uint32_t>& indices, bool index16)
{
    if (index16)
    {
        for (std::size_t i = 0; i < indices.size(); ++i)
            w.u16(static_cast<std::uint16_t>(indices[i]));
        return;
    }
    for (std::size_t i = 0; i < indices.size(); ++i) w.u32(indices[i]);
}

struct SurfaceSummary
{
    std::uint32_t attribs = 0;
    bool skinned = false;
};

bool writeSurface(Writer& w, std::uint32_t id, const SurfaceData& surface, SurfaceSummary& summary,
        ct::String& error)
{
    const std::size_t n = surface.positions.size();
    if (n == 0 || n > kMaxVertices)
        return bad(error, numbered("surface ", id, " has no vertices or too many"));
    if (static_cast<std::uint32_t>(surface.topology) > kMaxTopology)
        return bad(error, numbered("surface ", id, " has an unknown topology"));
    const bool hasNormal = !surface.normals.empty();
    const bool hasTangent = !surface.tangents.empty();
    const bool hasUv = !surface.uvs.empty();
    const bool hasUv2 = !surface.uvs2.empty();
    const bool hasColor = !surface.colors.empty();
    if ((hasNormal && surface.normals.size() != n) ||
            (hasTangent && surface.tangents.size() != n) || (hasUv && surface.uvs.size() != n) ||
            (hasUv2 && surface.uvs2.size() != n) || (hasColor && surface.colors.size() != n))
        return bad(error, numbered("surface ", id, " has an attribute with the wrong count"));
    const bool hasJoints = !surface.joints.empty();
    const bool hasWeights = !surface.weights.empty();
    if (hasJoints != hasWeights ||
            (hasJoints && (surface.joints.size() != n || surface.weights.size() != n)))
        return bad(error, numbered("surface ", id, " has inconsistent skin data"));
    if (!checkIndices(surface.indices, n))
        return bad(error, numbered("surface ", id, " has an index out of range"));

    std::uint32_t attribs = 0;
    if (hasNormal) attribs |= HasNormal;
    if (hasTangent) attribs |= HasTangent;
    if (hasUv) attribs |= HasUV;
    if (hasUv2) attribs |= HasUV2;
    if (hasColor) attribs |= HasColor;
    summary.attribs = attribs;
    summary.skinned = hasJoints;

    std::size_t marker = w.beginChunk(VertexBuffer);
    w.u32(id);
    w.u32(attribs);
    w.u32(static_cast<std::uint32_t>(n));
    w.u32(static_cast<std::uint32_t>(surface.topology));
    for (std::size_t i = 0; i < n; ++i)
    {
        w.floats(&surface.positions[i].x, 3);
        if (hasNormal) w.floats(&surface.normals[i].x, 3);
        if (hasTangent) w.floats(&surface.tangents[i].x, 4);
        if (hasUv) w.floats(&surface.uvs[i].x, 2);
        if (hasUv2) w.floats(&surface.uvs2[i].x, 2);
        if (hasColor) w.floats(&surface.colors[i].x, 4);
    }
    w.endChunk(marker);

    if (!surface.indices.empty())
    {
        const bool index16 = indicesFit16(surface.indices);
        marker = w.beginChunk(IndexBuffer);
        w.u32(id);
        w.u32(index16 ? static_cast<std::uint32_t>(Index16) : 0u);
        w.u32(static_cast<std::uint32_t>(surface.indices.size()));
        writeIndices(w, surface.indices, index16);
        w.endChunk(marker);
    }

    if (hasJoints)
    {
        marker = w.beginChunk(Skin);
        w.u32(id);
        w.u32(static_cast<std::uint32_t>(n));
        for (std::size_t i = 0; i < n; ++i)
        {
            w.u32(surface.joints[i]);
            w.floats(&surface.weights[i].x, 4);
        }
        w.endChunk(marker);
    }
    return true;
}

bool writeShadow(Writer& w, std::uint32_t mesh, const ct::Vector<SurfaceData>& surfaces,
        ct::String& error)
{
    for (std::size_t i = 0; i < surfaces.size(); ++i)
    {
        const SurfaceData& surface = surfaces[i];
        if (surface.positions.empty() || surface.positions.size() > kMaxVertices ||
                !checkIndices(surface.indices, surface.positions.size()) ||
                static_cast<std::uint32_t>(surface.topology) > kMaxTopology)
            return bad(error, numbered("shadow surface ", i, " is invalid"));
    }
    const std::size_t marker = w.beginChunk(ShadowMesh);
    w.u32(mesh);
    w.u32(static_cast<std::uint32_t>(surfaces.size()));
    for (std::size_t i = 0; i < surfaces.size(); ++i)
    {
        const SurfaceData& surface = surfaces[i];
        const bool index16 = indicesFit16(surface.indices);
        w.u32(static_cast<std::uint32_t>(surface.topology));
        w.u32(static_cast<std::uint32_t>(surface.positions.size()));
        w.u32(index16 ? static_cast<std::uint32_t>(Index16) : 0u);
        w.u32(static_cast<std::uint32_t>(surface.indices.size()));
        w.floats(&surface.positions[0].x, surface.positions.size() * 3);
        writeIndices(w, surface.indices, index16);
    }
    w.endChunk(marker);
    return true;
}

void writeStrings(Writer& w, std::uint32_t chunk, const ct::Vector<ct::String>& strings)
{
    const std::size_t marker = w.beginChunk(chunk);
    w.u32(static_cast<std::uint32_t>(strings.size()));
    for (std::size_t i = 0; i < strings.size(); ++i) w.string(strings[i]);
    w.endChunk(marker);
}

bool writeNodes(Writer& w, const ModelData& model, ct::String& error)
{
    const std::size_t count = model.nodes.size();
    if (count > kMaxNodes) return bad(error, "too many nodes");
    for (std::size_t i = 0; i < count; ++i)
    {
        const ModelNodeData& node = model.nodes[i];
        if (node.parent >= static_cast<int>(count) ||
                (node.mesh >= 0 && static_cast<std::size_t>(node.mesh) >= model.meshes.size()) ||
                (node.camera >= 0 &&
                        static_cast<std::size_t>(node.camera) >= model.cameras.size()) ||
                (node.skeleton >= 0 &&
                        static_cast<std::size_t>(node.skeleton) >= model.skeletons.size()))
            return bad(error, numbered("node ", i, " refers to something that does not exist"));
        for (std::size_t c = 0; c < node.children.size(); ++c)
        {
            const int child = node.children[c];
            if (child < 0 || static_cast<std::size_t>(child) >= count ||
                    model.nodes[static_cast<std::size_t>(child)].parent != static_cast<int>(i))
                return bad(error,
                        numbered("node ", i, " has a child list that disagrees with the parents"));
        }
    }
    for (std::size_t i = 0; i < model.roots.size(); ++i)
        if (model.roots[i] < 0 || static_cast<std::size_t>(model.roots[i]) >= count)
            return bad(error, "a root node does not exist");

    const std::size_t marker = w.beginChunk(Nodes);
    w.u32(static_cast<std::uint32_t>(count));
    for (std::size_t i = 0; i < count; ++i)
    {
        const ModelNodeData& node = model.nodes[i];
        w.string(node.name);
        w.s32(node.parent);
        w.s32(node.mesh);
        w.s32(node.camera);
        w.s32(node.skeleton);
        w.floats(&node.translation.x, 3);
        w.floats(&node.rotation.x, 4);
        w.floats(&node.scale.x, 3);
        w.u32(static_cast<std::uint32_t>(node.children.size()));
        for (std::size_t c = 0; c < node.children.size(); ++c) w.s32(node.children[c]);
    }
    w.u32(static_cast<std::uint32_t>(model.roots.size()));
    for (std::size_t i = 0; i < model.roots.size(); ++i) w.s32(model.roots[i]);
    w.endChunk(marker);
    return true;
}

void writeCameras(Writer& w, const ct::Vector<ModelCameraData>& cameras)
{
    const std::size_t marker = w.beginChunk(Cameras);
    w.u32(static_cast<std::uint32_t>(cameras.size()));
    for (std::size_t i = 0; i < cameras.size(); ++i)
    {
        const ModelCameraData& camera = cameras[i];
        w.u32(camera.perspective ? 1u : 0u);
        w.f32(camera.yfov);
        w.f32(camera.aspect);
        w.f32(camera.zNear);
        w.f32(camera.zFar);
        w.f32(camera.xmag);
        w.f32(camera.ymag);
    }
    w.endChunk(marker);
}

bool writeFile(const ct::String& path, const ct::Vector<std::uint8_t>& bytes, ct::String& error)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') slash = i;
    if (slash != ct::String::npos && slash != 0)
    {
        const ct::String directory(path.c_str(), slash);
        if (!fs_create_directory(directory.c_str()))
            return bad(error, ct::String("could not create '") + directory + "'");
    }
    const ct::String temporary = path + ".tmp";
    fs_remove_path(temporary.c_str());
    if (!file_write(temporary.c_str(), bytes.data(), bytes.size()))
        return bad(error, ct::String("could not write '") + temporary + "'");
    if (!fs_rename_path(temporary.c_str(), path.c_str()))
    {
        fs_remove_path(temporary.c_str());
        return bad(error, ct::String("could not replace '") + path + "'");
    }
    return true;
}

ct::String directoryPart(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') slash = i;
    if (slash == ct::String::npos) return ct::String();
    return ct::String(path.c_str(), slash + 1);
}

ct::String fileStemOf(const ct::String& path)
{
    std::size_t begin = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') begin = i + 1;
    std::size_t end = path.size();
    for (std::size_t i = path.size(); i > begin; --i)
    {
        if (path[i - 1] == '.')
        {
            end = i - 1;
            break;
        }
    }
    return ct::String(path.c_str() + begin, end - begin);
}

ct::String cleanName(const ct::String& name)
{
    ct::String result;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        result += (c == '"' || c == '\n' || c == '\r') ? '_' : c;
    }
    return result;
}

ct::String clipFileName(const ct::String& stem, const ct::String& clip, std::size_t index,
        const ct::Vector<ct::String>& taken)
{
    ct::String safe;
    for (std::size_t i = 0; i < clip.size(); ++i)
    {
        const char c = clip[i];
        const bool keep = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9') || c == '-' || c == '_';
        safe += keep ? c : '_';
    }
    if (safe.empty()) safe = numbered("clip_", index, "");
    const ct::String extension = ct::String(".") + kAnimationExtension;
    ct::String name = stem + "_" + safe + extension;
    for (std::size_t i = 0; i < taken.size(); ++i)
        if (taken[i] == name)
            return numbered((stem + "_" + safe + "_").c_str(), index, extension.c_str());
    return name;
}

void writeFloats(Writer& w, const ct::Vector<float>& values)
{
    if (!values.empty()) w.floats(values.data(), values.size());
}

void writeVec3s(Writer& w, const ct::Vector<Math::Vec3>& values)
{
    if (!values.empty()) w.floats(&values[0].x, values.size() * 3);
}

void writeQuats(Writer& w, const ct::Vector<Math::Quaternion>& values)
{
    if (!values.empty()) w.floats(&values[0].x, values.size() * 4);
}

void writeStringList(Writer& w, const ct::Vector<ct::String>& values)
{
    for (std::size_t i = 0; i < values.size(); ++i) w.string(values[i]);
}

bool writeClip(Writer& w, const AnimationClip& clip, const ct::Vector<Skeleton>& skeletons,
        ct::String& error)
{
    const int skeletonIndex = clip.skeleton();
    const Skeleton* bones =
            skeletonIndex >= 0 && static_cast<std::size_t>(skeletonIndex) < skeletons.size()
                    ? &skeletons[static_cast<std::size_t>(skeletonIndex)]
                    : nullptr;
    for (std::size_t i = 0; i < clip.tracks().size(); ++i)
    {
        const int bone = clip.tracks()[i].bone;
        if (bone < -1 ||
                (bone >= 0 && (!bones || static_cast<std::uint32_t>(bone) >= bones->boneCount())))
            return bad(error, ct::String("clip '") + clip.name() +
                                      "' has a bone track that is not in its skeleton");
    }

    const std::size_t marker = w.beginChunk(Clip);
    w.string(clip.name());
    w.f32(clip.duration());
    w.s32(skeletonIndex);
    w.u32(static_cast<std::uint32_t>(clip.loopMode()));
    w.u32(static_cast<std::uint32_t>(clip.tracks().size()));
    w.u32(static_cast<std::uint32_t>(clip.nodeTracks().size()));
    w.u32(static_cast<std::uint32_t>(clip.methodTracks().size()));
    w.u32(static_cast<std::uint32_t>(clip.valueTracks().size()));
    w.u32(static_cast<std::uint32_t>(clip.animationTracks().size()));
    w.u32(static_cast<std::uint32_t>(clip.events().size()));

    for (std::size_t i = 0; i < clip.tracks().size(); ++i)
    {
        const BoneTrack& track = clip.tracks()[i];
        w.s32(track.bone);
        w.string(track.bone >= 0 ? bones->bone(static_cast<std::uint32_t>(track.bone)).name
                                 : ct::String());
        w.u32(static_cast<std::uint32_t>(track.times.size()));
        w.u32(static_cast<std::uint32_t>(track.positions.size()));
        w.u32(static_cast<std::uint32_t>(track.rotations.size()));
        w.u32(static_cast<std::uint32_t>(track.scales.size()));
        writeFloats(w, track.times);
        writeVec3s(w, track.positions);
        writeQuats(w, track.rotations);
        writeVec3s(w, track.scales);
    }
    for (std::size_t i = 0; i < clip.nodeTracks().size(); ++i)
    {
        const NodeTrack& track = clip.nodeTracks()[i];
        w.string(track.path);
        w.u32(static_cast<std::uint32_t>(track.positionTimes.size()));
        w.u32(static_cast<std::uint32_t>(track.positions.size()));
        w.u32(static_cast<std::uint32_t>(track.rotationTimes.size()));
        w.u32(static_cast<std::uint32_t>(track.rotations.size()));
        w.u32(static_cast<std::uint32_t>(track.scaleTimes.size()));
        w.u32(static_cast<std::uint32_t>(track.scales.size()));
        writeFloats(w, track.positionTimes);
        writeVec3s(w, track.positions);
        writeFloats(w, track.rotationTimes);
        writeQuats(w, track.rotations);
        writeFloats(w, track.scaleTimes);
        writeVec3s(w, track.scales);
    }
    for (std::size_t i = 0; i < clip.methodTracks().size(); ++i)
    {
        const MethodTrack& track = clip.methodTracks()[i];
        w.string(track.path);
        w.u32(static_cast<std::uint32_t>(track.times.size()));
        w.u32(static_cast<std::uint32_t>(track.methods.size()));
        writeFloats(w, track.times);
        writeStringList(w, track.methods);
    }
    for (std::size_t i = 0; i < clip.valueTracks().size(); ++i)
    {
        const ValueTrack& track = clip.valueTracks()[i];
        w.string(track.path);
        w.string(track.property);
        w.u32(static_cast<std::uint32_t>(track.times.size()));
        w.u32(static_cast<std::uint32_t>(track.values.size()));
        writeFloats(w, track.times);
        writeFloats(w, track.values);
    }
    for (std::size_t i = 0; i < clip.animationTracks().size(); ++i)
    {
        const AnimationTrack& track = clip.animationTracks()[i];
        w.string(track.path);
        w.u32(static_cast<std::uint32_t>(track.times.size()));
        w.u32(static_cast<std::uint32_t>(track.animations.size()));
        writeFloats(w, track.times);
        writeStringList(w, track.animations);
    }
    for (std::size_t i = 0; i < clip.events().size(); ++i)
    {
        w.f32(clip.events()[i].time);
        w.string(clip.events()[i].name);
    }
    w.endChunk(marker);
    return true;
}

} // namespace

bool buildPMeshSkeleton(const Skeleton& skeleton, ct::Vector<std::uint8_t>& out, ct::String& error)
{
    out.clear();
    if (skeleton.boneCount() > kMaxBones) return bad(error, "too many bones");
    Writer w(out);
    w.header(kSkeletonMagic, 0);
    const std::size_t marker = w.beginChunk(Bones);
    w.u32(skeleton.boneCount());
    for (std::uint32_t i = 0; i < skeleton.boneCount(); ++i)
    {
        const Bone& bone = skeleton.bone(i);
        w.string(bone.name);
        w.s32(bone.parent);
        w.mat4(bone.bindLocal);
        w.mat4(bone.inverseBind);
    }
    w.endChunk(marker);
    const std::size_t end = w.beginChunk(End);
    w.endChunk(end);
    return true;
}

bool buildPMeshAnimation(const AnimationClip& clip, const ct::Vector<Skeleton>& skeletons,
        ct::Vector<std::uint8_t>& out, ct::String& error)
{
    out.clear();
    Writer w(out);
    w.header(kAnimationMagic, 0);
    if (!writeClip(w, clip, skeletons, error)) return false;
    const std::size_t end = w.beginChunk(End);
    w.endChunk(end);
    return true;
}

bool buildPMesh(const ModelData& model, const ModelImporter& importer, const ct::String& fileStem,
        const PMeshSaveOptions& options, PMeshFiles& out, ct::String& error)
{
    out = PMeshFiles();
    error.clear();

    ct::Vector<ct::String> materialNames;
    if (options.materialNames)
    {
        if (options.materialNames->size() != model.materials.size())
            return bad(error, "material name list does not match the materials");
        materialNames = *options.materialNames;
    }
    else
    {
        for (std::size_t i = 0; i < model.materials.size(); ++i)
            materialNames.push_back(model.materials[i].name);
    }
    if (model.meshes.size() > kMaxMeshes) return bad(error, "too many meshes");

    Writer w(out.mesh);
    w.header(kMeshMagic, 0);

    std::size_t marker = w.beginChunk(MaterialSlots);
    w.u32(static_cast<std::uint32_t>(materialNames.size()));
    for (std::size_t i = 0; i < materialNames.size(); ++i) w.string(materialNames[i]);
    w.endChunk(marker);

    ct::Vector<ct::Vector<SubMeshRecord>> records;
    records.resize(model.meshes.size());
    std::uint32_t surfaceId = 0;
    std::uint32_t attribUnion = 0;
    bool anySkin = false;
    for (std::size_t m = 0; m < model.meshes.size(); ++m)
    {
        const ModelMeshData& mesh = model.meshes[m];
        if (mesh.primitives.size() > kMaxSubMeshes) return bad(error, "too many sub-meshes");
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
        {
            const ModelPrimitiveData& primitive = mesh.primitives[p];
            if (surfaceId >= kMaxSurfaces) return bad(error, "too many surfaces");
            if (primitive.material >= 0 &&
                    static_cast<std::size_t>(primitive.material) >= model.materials.size())
                return bad(error,
                        numbered("surface ", surfaceId, " has a material that does not exist"));

            SurfaceData surface;
            if (!importer.readPrimitive(model, primitive, surface))
                return bad(error,
                        numbered("could not read surface ", surfaceId, " from the model"));
            if (options.generateTangents && surface.tangents.empty() &&
                    surface.topology == prisma::Topology::Triangles && !surface.normals.empty() &&
                    !surface.uvs.empty())
                MeshStore::generateTangents(surface);

            SurfaceSummary summary;
            if (!writeSurface(w, surfaceId, surface, summary, error)) return false;
            if (primitive.skinned && !summary.skinned)
                return bad(error, numbered("surface ", surfaceId, " is skinned but has no joints"));
            Math::Box bounds =
                    primitive.hasAabb ? primitive.aabb : MeshStore::computeAabb(surface.positions);
            if (!validBox(bounds))
                return bad(error, numbered("surface ", surfaceId, " has invalid bounds"));

            SubMeshRecord record;
            record.surface = surfaceId;
            record.materialSlot = primitive.material < 0
                                          ? kNoSlot
                                          : static_cast<std::uint32_t>(primitive.material);
            record.flags = (primitive.skinned ? static_cast<std::uint32_t>(SubMeshSkinned) : 0u) |
                           (primitive.hasAabb ? static_cast<std::uint32_t>(SubMeshHasBounds) : 0u);
            record.boundsMin[0] = bounds.min.x;
            record.boundsMin[1] = bounds.min.y;
            record.boundsMin[2] = bounds.min.z;
            record.boundsMax[0] = bounds.max.x;
            record.boundsMax[1] = bounds.max.y;
            record.boundsMax[2] = bounds.max.z;
            records[m].push_back(record);
            attribUnion |= summary.attribs;
            anySkin = anySkin || summary.skinned;
            ++surfaceId;
        }
    }

    marker = w.beginChunk(SubMeshes);
    w.u32(static_cast<std::uint32_t>(model.meshes.size()));
    for (std::size_t m = 0; m < model.meshes.size(); ++m)
    {
        w.string(model.meshes[m].name);
        w.u32(static_cast<std::uint32_t>(records[m].size()));
        for (std::size_t i = 0; i < records[m].size(); ++i)
        {
            const SubMeshRecord& record = records[m][i];
            w.u32(record.surface);
            w.u32(record.materialSlot);
            w.u32(record.flags);
            w.floats(record.boundsMin, 3);
            w.floats(record.boundsMax, 3);
        }
    }
    w.endChunk(marker);

    if (!model.nodes.empty() || !model.roots.empty())
        if (!writeNodes(w, model, error)) return false;
    if (!model.cameras.empty()) writeCameras(w, model.cameras);
    for (std::size_t m = 0; m < model.meshes.size(); ++m)
        if (!model.meshes[m].shadowSurfaces.empty() &&
                !writeShadow(w, static_cast<std::uint32_t>(m), model.meshes[m].shadowSurfaces,
                        error))
            return false;

    if (!model.skeletons.empty())
    {
        if (!options.skeletonFiles.empty() &&
                options.skeletonFiles.size() != model.skeletons.size())
            return bad(error, "skeleton file list does not match the skeletons");
        for (std::size_t s = 0; s < model.skeletons.size(); ++s)
        {
            ct::String name;
            if (!options.skeletonFiles.empty()) name = options.skeletonFiles[s];
            else if (model.skeletons.size() == 1)
                name = fileStem + "." + kSkeletonExtension;
            else
                name = numbered((fileStem + "_").c_str(), s,
                        (ct::String(".") + kSkeletonExtension).c_str());
            if (name.empty()) return bad(error, "empty skeleton file name");
            ct::Vector<std::uint8_t> bytes;
            if (!buildPMeshSkeleton(model.skeletons[s], bytes, error)) return false;
            out.skeletonNames.push_back(name);
            out.skeletons.push_back(static_cast<ct::Vector<std::uint8_t>&&>(bytes));
        }
        writeStrings(w, SkeletonLink, out.skeletonNames);
    }
    if (!model.animations.empty())
    {
        if (model.animations.size() > kMaxLinks) return bad(error, "too many clips");
        for (std::size_t i = 0; i < model.animations.size(); ++i)
        {
            ct::Vector<std::uint8_t> bytes;
            if (!buildPMeshAnimation(model.animations[i], model.skeletons, bytes, error))
                return false;
            out.animationNames.push_back(
                    clipFileName(fileStem, model.animations[i].name(), i, out.animationNames));
            out.animations.push_back(static_cast<ct::Vector<std::uint8_t>&&>(bytes));
        }
        writeStrings(w, AnimationLink, out.animationNames);
    }

    const std::size_t end = w.beginChunk(End);
    w.endChunk(end);

    const std::uint32_t flags = LittleEndian |
                                (anySkin ? static_cast<std::uint32_t>(Skinned) : 0u) |
                                (attribUnion << kHeaderAttribShift);
    for (int i = 0; i < 4; ++i) out.mesh[8 + i] = static_cast<std::uint8_t>(flags >> (8 * i));
    return true;
}

bool savePMesh(const ModelData& model, const ModelImporter& importer, const ct::String& path,
        const PMeshSaveOptions& options, ct::String& error)
{
    PMeshFiles files;
    if (!buildPMesh(model, importer, fileStemOf(path), options, files, error)) return false;
    const ct::String directory = directoryPart(path);
    if (!writeFile(path, files.mesh, error)) return false;
    for (std::size_t i = 0; i < files.skeletons.size(); ++i)
        if (!writeFile(directory + files.skeletonNames[i], files.skeletons[i], error)) return false;
    for (std::size_t i = 0; i < files.animations.size(); ++i)
        if (!writeFile(directory + files.animationNames[i], files.animations[i], error))
            return false;
    return true;
}

bool convertModelToPMesh(AssetLoader& loader, MaterialStore& materials, const ct::String& source,
        const ct::String& destination, const PMeshConvertOptions& options, ct::String& error)
{
    const ModelId model = loader.requestModel(source);
    if (!model.valid()) return bad(error, "could not request the model");
    loader.waitAll();
    const ModelInfo* info = loader.model(model);
    if (!info || info->status != LoadStatus::Loaded || !info->importer)
    {
        const ct::String reason =
                info && !info->error.empty() ? info->error : ct::String("load failed");
        loader.freeModel(model);
        return bad(error, ct::String("could not load '") + source + "': " + reason);
    }

    ct::String target = destination;
    if (target.empty())
    {
        std::size_t end = source.size();
        std::size_t begin = 0;
        for (std::size_t i = 0; i < source.size(); ++i)
            if (source[i] == '/' || source[i] == '\\') begin = i + 1;
        for (std::size_t i = source.size(); i > begin; --i)
            if (source[i - 1] == '.')
            {
                end = i - 1;
                break;
            }
        target = ct::String(source.c_str(), end) + "." + kMeshExtension;
    }

    ct::Vector<ct::String> names;
    for (std::size_t i = 0; i < info->materials.size(); ++i)
    {
        const MaterialInfo* material = materials.get(info->materials[i]);
        ct::String name = material ? cleanName(material->name) : ct::String();
        if (name.empty()) name = numbered("material_", i, "");
        names.push_back(name);
    }

    PMeshSaveOptions save;
    save.materialNames = &names;
    save.generateTangents = options.bakeTangents;
    bool ok = savePMesh(info->document, *info->importer, target, save, error);
    if (ok) ok = saveMaterials(materials, info->materials, materialSidecarPath(target), error);
    loader.freeModel(model);
    return ok;
}

bool editModelToPMesh(AssetLoader& loader, MaterialStore& materials, const ct::String& source,
        const PMeshSurfaceEdit& edit, const ct::String& destination,
        ct::Vector<ct::String>& extracted, ct::String& error)
{
    extracted.clear();
    if (destination.empty()) return bad(error, "the edited mesh needs a file");
    const ct::String extension = ct::String(".") + kMeshExtension;
    const bool pmeshSource = source.ends_with(extension.c_str());
    const bool inPlace = destination == source;
    if (inPlace && !pmeshSource) return bad(error, "only a .pmesh file can be written over");
    const ct::String written = inPlace ? destination + ".tmp" : destination;

    ModelLoader* importers = ModelLoader::singleton();
    ModelImporter* importer = importers ? importers->importerFor(source) : nullptr;
    if (!importer) return bad(error, ct::String("no importer reads '") + source + "'");
    ct::Vector<ct::String> moved;
    {
        ModelData document;
        if (!importer->import(source, document))
            return bad(error, ct::String("could not read '") + source + "': " + document.error);

        int target = edit.mesh.empty() && !document.meshes.empty() ? 0 : -1;
        for (std::size_t i = 0; i < document.meshes.size() && target < 0; ++i)
            if (document.meshes[i].name == edit.mesh) target = static_cast<int>(i);
        if (target < 0) return bad(error, ct::String("the model has no mesh '") + edit.mesh + "'");

        ModelMeshData& mesh = document.meshes[static_cast<std::size_t>(target)];
        const std::size_t count = mesh.primitives.size();
        ct::Vector<std::uint8_t> fate;
        fate.resize(count);
        for (std::size_t i = 0; i < count; ++i) fate[i] = 0;
        for (std::size_t i = 0; i < edit.remove.size(); ++i)
        {
            if (edit.remove[i] >= count) return bad(error, "a removed surface does not exist");
            fate[edit.remove[i]] = 1;
        }
        for (std::size_t i = 0; i < edit.extract.size(); ++i)
        {
            if (edit.extract[i] >= count) return bad(error, "an extracted surface does not exist");
            if (fate[edit.extract[i]] != 0)
                return bad(error, "a surface is both removed and extracted");
            fate[edit.extract[i]] = 2;
        }
        std::size_t kept = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            if (mesh.primitives[i].skinned) return bad(error, "only static meshes can be edited");
            if (fate[i] == 0) ++kept;
        }
        if (kept == 0) return bad(error, "the mesh would have no surface left");

        const ct::String meshName = mesh.name;
        ct::Vector<ModelPrimitiveData> remaining;
        ct::Vector<ModelMeshData> pieces;
        for (std::size_t i = 0; i < count; ++i)
        {
            if (fate[i] == 0) remaining.push_back(mesh.primitives[i]);
            if (fate[i] != 2) continue;
            ModelMeshData piece;
            piece.name = meshName + numbered("_surface", i, "");
            bool taken = true;
            while (taken)
            {
                taken = false;
                for (std::size_t k = 0; k < document.meshes.size() && !taken; ++k)
                    taken = document.meshes[k].name == piece.name;
                for (std::size_t k = 0; k < pieces.size() && !taken; ++k)
                    taken = pieces[k].name == piece.name;
                if (taken) piece.name += "_";
            }
            piece.primitives.push_back(mesh.primitives[i]);
            pieces.push_back(piece);
        }
        mesh.primitives = remaining;
        mesh.shadowSurfaces.clear();
        for (std::size_t i = 0; i < pieces.size(); ++i)
        {
            moved.push_back(pieces[i].name);
            document.meshes.push_back(pieces[i]);
        }

        PMeshSaveOptions save;
        save.generateTangents = true;
        ct::Vector<ct::String> names;
        ModelId model;
        const ModelInfo* info = nullptr;
        if (!pmeshSource)
        {
            model = loader.requestModel(source);
            if (!model.valid()) return bad(error, "could not request the model");
            loader.waitAll();
            info = loader.model(model);
            if (!info || info->status != LoadStatus::Loaded)
            {
                loader.freeModel(model);
                return bad(error, ct::String("could not load '") + source + "'");
            }
            for (std::size_t i = 0; i < info->materials.size(); ++i)
            {
                const MaterialInfo* material = materials.get(info->materials[i]);
                ct::String name = material ? cleanName(material->name) : ct::String();
                if (name.empty()) name = numbered("material_", i, "");
                names.push_back(name);
            }
            save.materialNames = &names;
        }
        bool ok = savePMesh(document, *importer, written, save, error);
        if (ok && info)
            ok = saveMaterials(materials, info->materials, materialSidecarPath(destination), error);
        if (model.valid()) loader.freeModel(model);
        if (!ok)
        {
            if (inPlace) fs_remove_path(written.c_str());
            return false;
        }
    }

    if (pmeshSource && !inPlace)
    {
        const ct::String from = materialSidecarPath(source);
        PathInfo found;
        if (fs_get_path_info(from.c_str(), &found) && found.type == PATH_TYPE_FILE)
        {
            const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(from);
            const ct::String to = materialSidecarPath(destination);
            if (!FileSystem::writeBinary(to, bytes.data(), bytes.size()))
                return bad(error, ct::String("could not write '") + to + "'");
        }
    }
    if (inPlace && !fs_rename_path(written.c_str(), destination.c_str()))
    {
        fs_remove_path(written.c_str());
        return bad(error, ct::String("could not replace '") + destination + "'");
    }
    extracted = moved;
    return true;
}

} // namespace prisma3d
