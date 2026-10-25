#include "prisma3d/import/PMeshImporter.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/import/PMeshFormat.h"
#include "prisma3d/import/PMeshIO.h"

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

struct SurfaceRef
{
    std::size_t vertexOffset = 0;
    std::uint32_t attribs = 0;
    std::uint32_t vertexCount = 0;
    std::uint32_t topology = 0;
    std::size_t indexOffset = 0;
    std::uint32_t indexCount = 0;
    bool index16 = false;
    bool hasIndices = false;
    std::size_t skinOffset = 0;
    bool hasSkin = false;
};

struct PMeshSource : ModelSource
{
    ct::Vector<std::uint8_t> bytes;
    ct::Vector<SurfaceRef> surfaces;
};

struct MeshEntry
{
    ct::String name;
    ct::Vector<SubMeshRecord> subMeshes;
};

struct ShadowEntry
{
    std::uint32_t mesh = 0;
    ct::Vector<SurfaceData> surfaces;
};

struct ParsedFile
{
    std::uint32_t flags = 0;
    ct::Vector<ct::String> materials;
    bool hasMaterials = false;
    ct::Vector<MeshEntry> meshes;
    bool hasSubMeshes = false;
    ct::Vector<ModelNodeData> nodes;
    ct::Vector<int> roots;
    bool hasNodes = false;
    ct::Vector<ModelCameraData> cameras;
    bool hasCameras = false;
    ct::Vector<ShadowEntry> shadows;
    ct::Vector<ct::String> skeletonLinks;
    bool hasSkeletonLinks = false;
    ct::Vector<ct::String> animationLinks;
    bool hasAnimationLinks = false;
};

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') slash = i;
    if (slash == ct::String::npos) return ct::String();
    return ct::String(path.c_str(), slash + 1);
}

bool readStrings(Reader& r, ct::Vector<ct::String>& out, std::uint32_t limit)
{
    std::uint32_t count = 0;
    if (!r.u32(count)) return false;
    if (count > limit || !r.fits(count, 4)) return r.fail("invalid string count");
    out.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
        if (!r.string(out[i])) return false;
    return true;
}

bool readVertexBuffer(Reader& r, ct::Vector<SurfaceRef>& surfaces)
{
    VertexBufferHeader h;
    if (!r.u32(h.surface) || !r.u32(h.attribs) || !r.u32(h.vertexCount) || !r.u32(h.topology))
        return false;
    if (h.surface != surfaces.size()) return r.fail("surface ids must be sequential");
    if (surfaces.size() >= kMaxSurfaces) return r.fail("too many surfaces");
    if ((h.attribs & ~kKnownAttribs) != 0) return r.fail("unknown vertex attributes");
    if (h.vertexCount == 0 || h.vertexCount > kMaxVertices) return r.fail("invalid vertex count");
    if (h.topology > kMaxTopology) return r.fail("invalid topology");
    if (!r.fits(h.vertexCount, vertexAttribStride(h.attribs)))
        return r.fail("truncated vertex data");
    SurfaceRef ref;
    ref.vertexOffset = r.tell();
    ref.attribs = h.attribs;
    ref.vertexCount = h.vertexCount;
    ref.topology = h.topology;
    surfaces.push_back(ref);
    return true;
}

bool readIndexBuffer(Reader& r, ct::Vector<SurfaceRef>& surfaces)
{
    IndexBufferHeader h;
    if (!r.u32(h.surface) || !r.u32(h.flags) || !r.u32(h.indexCount)) return false;
    if (h.surface >= surfaces.size()) return r.fail("index buffer of an unknown surface");
    SurfaceRef& ref = surfaces[h.surface];
    if (ref.hasIndices) return r.fail("surface has two index buffers");
    if ((h.flags & ~static_cast<std::uint32_t>(Index16)) != 0) return r.fail("unknown index flags");
    if (h.indexCount == 0 || h.indexCount > kMaxIndices) return r.fail("invalid index count");
    const bool index16 = (h.flags & Index16) != 0;
    if (!r.fits(h.indexCount, index16 ? 2 : 4)) return r.fail("truncated index data");
    ref.hasIndices = true;
    ref.index16 = index16;
    ref.indexCount = h.indexCount;
    ref.indexOffset = r.tell();
    return true;
}

bool readSkinChunk(Reader& r, ct::Vector<SurfaceRef>& surfaces)
{
    SkinHeader h;
    if (!r.u32(h.surface) || !r.u32(h.vertexCount)) return false;
    if (h.surface >= surfaces.size()) return r.fail("skin of an unknown surface");
    SurfaceRef& ref = surfaces[h.surface];
    if (ref.hasSkin) return r.fail("surface has two skin chunks");
    if (h.vertexCount != ref.vertexCount)
        return r.fail("skin vertex count differs from the surface");
    if (!r.fits(h.vertexCount, kSkinVertexSize)) return r.fail("truncated skin data");
    ref.hasSkin = true;
    ref.skinOffset = r.tell();
    return true;
}

bool readSubMeshes(Reader& r, ParsedFile& file)
{
    std::uint32_t meshCount = 0;
    if (!r.u32(meshCount)) return false;
    if (meshCount > kMaxMeshes || !r.fits(meshCount, 8)) return r.fail("invalid mesh count");
    file.meshes.resize(meshCount);
    for (std::uint32_t m = 0; m < meshCount; ++m)
    {
        MeshEntry& mesh = file.meshes[m];
        std::uint32_t count = 0;
        if (!r.string(mesh.name) || !r.u32(count)) return false;
        if (count > kMaxSubMeshes || !r.fits(count, kSubMeshRecordSize))
            return r.fail("invalid sub-mesh count");
        mesh.subMeshes.resize(count);
        for (std::uint32_t i = 0; i < count; ++i)
        {
            SubMeshRecord& record = mesh.subMeshes[i];
            Math::Vec3 low;
            Math::Vec3 high;
            if (!r.u32(record.surface) || !r.u32(record.materialSlot) || !r.u32(record.flags) ||
                    !r.box(low, high))
                return false;
            record.boundsMin[0] = low.x;
            record.boundsMin[1] = low.y;
            record.boundsMin[2] = low.z;
            record.boundsMax[0] = high.x;
            record.boundsMax[1] = high.y;
            record.boundsMax[2] = high.z;
        }
    }
    return true;
}

bool readNodes(Reader& r, ParsedFile& file)
{
    std::uint32_t count = 0;
    if (!r.u32(count)) return false;
    if (count > kMaxNodes || !r.fits(count, kNodeFixedSize)) return r.fail("invalid node count");
    file.nodes.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        ModelNodeData& node = file.nodes[i];
        std::int32_t parent = 0;
        std::int32_t mesh = 0;
        std::int32_t camera = 0;
        std::int32_t skeleton = 0;
        if (!r.string(node.name) || !r.s32(parent) || !r.s32(mesh) || !r.s32(camera) ||
                !r.s32(skeleton))
            return false;
        if (parent < -1 || mesh < -1 || camera < -1 || skeleton < -1)
            return r.fail("negative reference in node");
        node.parent = parent;
        node.mesh = mesh;
        node.camera = camera;
        node.skeleton = skeleton;
        if (!r.vec3(node.translation) || !r.f32Finite(node.rotation.x) ||
                !r.f32Finite(node.rotation.y) || !r.f32Finite(node.rotation.z) ||
                !r.f32Finite(node.rotation.w) || !r.vec3(node.scale))
            return false;
        std::uint32_t childCount = 0;
        if (!r.u32(childCount)) return false;
        if (!r.fits(childCount, 4)) return r.fail("invalid child count");
        node.children.resize(childCount);
        for (std::uint32_t c = 0; c < childCount; ++c)
        {
            std::int32_t child = 0;
            if (!r.s32(child)) return false;
            if (child < 0) return r.fail("negative child reference");
            node.children[c] = child;
        }
    }
    std::uint32_t rootCount = 0;
    if (!r.u32(rootCount)) return false;
    if (!r.fits(rootCount, 4)) return r.fail("invalid root count");
    file.roots.resize(rootCount);
    for (std::uint32_t i = 0; i < rootCount; ++i)
    {
        std::int32_t root = 0;
        if (!r.s32(root)) return false;
        if (root < 0) return r.fail("negative root reference");
        file.roots[i] = root;
    }
    return true;
}

bool readCameras(Reader& r, ParsedFile& file)
{
    std::uint32_t count = 0;
    if (!r.u32(count)) return false;
    if (count > kMaxNodes || !r.fits(count, kCameraRecordSize))
        return r.fail("invalid camera count");
    file.cameras.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        ModelCameraData& camera = file.cameras[i];
        std::uint32_t perspective = 0;
        if (!r.u32(perspective) || !r.f32Finite(camera.yfov) || !r.f32Finite(camera.aspect) ||
                !r.f32Finite(camera.zNear) || !r.f32Finite(camera.zFar) ||
                !r.f32Finite(camera.xmag) || !r.f32Finite(camera.ymag))
            return false;
        camera.perspective = perspective != 0;
    }
    return true;
}

bool readShadow(Reader& r, ParsedFile& file)
{
    ShadowEntry entry;
    std::uint32_t count = 0;
    if (!r.u32(entry.mesh) || !r.u32(count)) return false;
    if (count == 0 || count > kMaxSurfaces || !r.fits(count, kShadowSurfaceHeaderSize))
        return r.fail("invalid shadow surface count");
    entry.surfaces.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        SurfaceData& surface = entry.surfaces[i];
        std::uint32_t topology = 0;
        std::uint32_t vertexCount = 0;
        std::uint32_t flags = 0;
        std::uint32_t indexCount = 0;
        if (!r.u32(topology) || !r.u32(vertexCount) || !r.u32(flags) || !r.u32(indexCount))
            return false;
        if (topology > kMaxTopology) return r.fail("invalid topology");
        if (vertexCount == 0 || vertexCount > kMaxVertices || indexCount > kMaxIndices ||
                (flags & ~static_cast<std::uint32_t>(Index16)) != 0)
            return r.fail("invalid shadow surface");
        const bool index16 = (flags & Index16) != 0;
        if (!r.fits(vertexCount, 12)) return r.fail("truncated shadow positions");
        surface.topology = static_cast<prisma::Topology>(topology);
        surface.positions.resize(vertexCount);
        if (!r.floats(&surface.positions[0].x, static_cast<std::size_t>(vertexCount) * 3))
            return false;
        if (indexCount != 0)
        {
            if (!r.fits(indexCount, index16 ? 2 : 4)) return r.fail("truncated shadow indices");
            surface.indices.resize(indexCount);
            if (index16)
            {
                for (std::uint32_t k = 0; k < indexCount; ++k)
                {
                    std::uint16_t value = 0;
                    if (!r.u16(value)) return false;
                    surface.indices[k] = value;
                }
            }
            else if (!r.words(surface.indices.data(), indexCount))
                return false;
            for (std::uint32_t k = 0; k < indexCount; ++k)
                if (surface.indices[k] >= vertexCount) return r.fail("shadow index out of range");
        }
    }
    file.shadows.push_back(static_cast<ShadowEntry&&>(entry));
    return true;
}

bool parseMesh(const std::uint8_t* data, std::size_t size, ct::Vector<SurfaceRef>& surfaces,
        ParsedFile& file, ct::String& error)
{
    Reader r(data, size);
    bool ok = r.header(kMeshMagic, file.flags);
    bool sawEnd = false;
    while (ok)
    {
        if (r.remaining() == 0)
        {
            ok = r.fail("missing end chunk, the file is truncated");
            break;
        }
        std::uint32_t id = 0;
        std::uint64_t chunkSize = 0;
        if (!r.next(id, chunkSize) || !r.enter(chunkSize))
        {
            ok = false;
            break;
        }
        switch (id)
        {
            case MaterialSlots:
                ok = !file.hasMaterials || r.fail("duplicate material chunk");
                ok = ok && readStrings(r, file.materials, kMaxMeshes);
                file.hasMaterials = true;
                break;
            case VertexBuffer:
                ok = readVertexBuffer(r, surfaces);
                break;
            case IndexBuffer:
                ok = readIndexBuffer(r, surfaces);
                break;
            case Skin:
                ok = readSkinChunk(r, surfaces);
                break;
            case SubMeshes:
                ok = !file.hasSubMeshes || r.fail("duplicate sub-mesh chunk");
                ok = ok && readSubMeshes(r, file);
                file.hasSubMeshes = true;
                break;
            case Nodes:
                ok = !file.hasNodes || r.fail("duplicate node chunk");
                ok = ok && readNodes(r, file);
                file.hasNodes = true;
                break;
            case Cameras:
                ok = !file.hasCameras || r.fail("duplicate camera chunk");
                ok = ok && readCameras(r, file);
                file.hasCameras = true;
                break;
            case ShadowMesh:
                ok = readShadow(r, file);
                break;
            case SkeletonLink:
                ok = !file.hasSkeletonLinks || r.fail("duplicate skeleton link chunk");
                ok = ok && readStrings(r, file.skeletonLinks, kMaxLinks);
                file.hasSkeletonLinks = true;
                break;
            case AnimationLink:
                ok = !file.hasAnimationLinks || r.fail("duplicate animation link chunk");
                ok = ok && readStrings(r, file.animationLinks, kMaxLinks);
                file.hasAnimationLinks = true;
                break;
            case End:
                sawEnd = true;
                break;
            default:
                break;
        }
        r.leave();
        if (ok && sawEnd) break;
    }
    if (ok && r.remaining() != 0) ok = r.fail("data after the end chunk");
    if (!ok) error = r.error();
    return ok;
}

bool checkReferences(const ParsedFile& file, const ct::Vector<SurfaceRef>& surfaces,
        ct::String& error)
{
    std::uint32_t attribs = 0;
    bool skinned = false;
    for (std::size_t i = 0; i < surfaces.size(); ++i)
    {
        attribs |= surfaces[i].attribs;
        skinned = skinned || surfaces[i].hasSkin;
    }
    if (((file.flags >> kHeaderAttribShift) & kKnownAttribs) != attribs ||
            ((file.flags & Skinned) != 0) != skinned)
    {
        error = "header flags disagree with the surfaces";
        return false;
    }

    for (std::size_t m = 0; m < file.meshes.size(); ++m)
    {
        for (std::size_t i = 0; i < file.meshes[m].subMeshes.size(); ++i)
        {
            const SubMeshRecord& record = file.meshes[m].subMeshes[i];
            if (record.surface >= surfaces.size())
            {
                error = "sub-mesh refers to a missing surface";
                return false;
            }
            if (record.materialSlot != kNoSlot && record.materialSlot >= file.materials.size())
            {
                error = "sub-mesh refers to a missing material slot";
                return false;
            }
            if ((record.flags & SubMeshSkinned) != 0 && !surfaces[record.surface].hasSkin)
            {
                error = "skinned sub-mesh has a surface without skin data";
                return false;
            }
        }
    }
    for (std::size_t i = 0; i < file.shadows.size(); ++i)
    {
        if (file.shadows[i].mesh >= file.meshes.size())
        {
            error = "shadow mesh of a missing mesh";
            return false;
        }
        for (std::size_t k = 0; k < i; ++k)
            if (file.shadows[k].mesh == file.shadows[i].mesh)
            {
                error = "two shadow meshes for one mesh";
                return false;
            }
    }

    const std::size_t nodeCount = file.nodes.size();
    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        const ModelNodeData& node = file.nodes[i];
        if (node.parent >= static_cast<int>(nodeCount) ||
                (node.mesh >= 0 && static_cast<std::size_t>(node.mesh) >= file.meshes.size()) ||
                (node.camera >= 0 &&
                        static_cast<std::size_t>(node.camera) >= file.cameras.size()) ||
                (node.skeleton >= 0 &&
                        static_cast<std::size_t>(node.skeleton) >= file.skeletonLinks.size()))
        {
            error = "node refers to something that does not exist";
            return false;
        }
        for (std::size_t c = 0; c < node.children.size(); ++c)
        {
            const int child = node.children[c];
            if (static_cast<std::size_t>(child) >= nodeCount ||
                    file.nodes[static_cast<std::size_t>(child)].parent != static_cast<int>(i))
            {
                error = "node child list disagrees with the parents";
                return false;
            }
        }
    }
    for (std::size_t i = 0; i < file.roots.size(); ++i)
    {
        if (static_cast<std::size_t>(file.roots[i]) >= nodeCount ||
                file.nodes[static_cast<std::size_t>(file.roots[i])].parent != -1)
        {
            error = "root is not a parentless node";
            return false;
        }
    }
    ct::Vector<std::uint8_t> state;
    state.resize(nodeCount, 0);
    for (std::size_t i = 0; i < nodeCount; ++i)
    {
        if (state[i] == 2) continue;
        std::size_t current = i;
        for (;;)
        {
            if (state[current] == 1)
            {
                error = "node hierarchy has a cycle";
                return false;
            }
            if (state[current] == 2) break;
            state[current] = 1;
            const int parent = file.nodes[current].parent;
            if (parent < 0) break;
            current = static_cast<std::size_t>(parent);
        }
        current = i;
        while (state[current] == 1)
        {
            state[current] = 2;
            const int parent = file.nodes[current].parent;
            if (parent < 0) break;
            current = static_cast<std::size_t>(parent);
        }
    }
    return true;
}

bool loadLinked(const ct::String& meshPath, const ct::String& link, ct::Vector<std::uint8_t>& bytes)
{
    ct::String normalized = link;
    for (std::size_t i = 0; i < normalized.size(); ++i)
        if (normalized[i] == '\\') normalized[i] = '/';
    const FileSystem& files = FileSystem::getSingleton();
    const ct::String besideMesh = directoryOf(meshPath) + normalized;
    const ct::String candidates[2] = { besideMesh, normalized };
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (!files.exists(candidates[i])) continue;
        bytes = files.readBinary(candidates[i]);
        return !bytes.empty();
    }
    return false;
}

std::uint64_t surfaceBytes(const SurfaceRef& surface)
{
    std::uint64_t perVertex = 12;
    if (surface.attribs & HasNormal) perVertex += 12;
    if (surface.attribs & HasTangent) perVertex += 16;
    if (surface.attribs & HasUV) perVertex += 8;
    if (surface.attribs & HasUV2) perVertex += 8;
    if (surface.attribs & HasColor) perVertex += 16;
    if (surface.hasSkin) perVertex += 20;
    return perVertex * surface.vertexCount + static_cast<std::uint64_t>(surface.indexCount) * 4;
}

bool fail(ModelData& out, const ct::String& message)
{
    out.error = ct::String("pmesh: ") + message;
    return false;
}

bool build(ct::Vector<std::uint8_t>&& bytes, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    ct::Unique<PMeshSource> owned = ct::make_unique<PMeshSource>();
    owned->bytes = static_cast<ct::Vector<std::uint8_t>&&>(bytes);

    ParsedFile file;
    ct::String error;
    if (!parseMesh(owned->bytes.data(), owned->bytes.size(), owned->surfaces, file, error) ||
            !checkReferences(file, owned->surfaces, error))
        return fail(out, error);

    for (std::size_t i = 0; i < file.skeletonLinks.size(); ++i)
    {
        ct::Vector<std::uint8_t> linked;
        if (file.skeletonLinks[i].empty() || !loadLinked(path, file.skeletonLinks[i], linked))
            return fail(out,
                    ct::String("linked skeleton '") + file.skeletonLinks[i] + "' is missing");
        Skeleton skeleton;
        if (!readPMeshSkeleton(linked.data(), linked.size(), skeleton, error))
            return fail(out,
                    ct::String("linked skeleton '") + file.skeletonLinks[i] + "': " + error);
        out.skeletons.push_back(static_cast<Skeleton&&>(skeleton));
    }
    for (std::size_t i = 0; i < file.animationLinks.size(); ++i)
    {
        ct::Vector<std::uint8_t> linked;
        if (file.animationLinks[i].empty() || !loadLinked(path, file.animationLinks[i], linked))
            return fail(out,
                    ct::String("linked animation '") + file.animationLinks[i] + "' is missing");
        if (!readPMeshAnimations(linked.data(), linked.size(), out.skeletons, out.animations,
                    error))
            return fail(out,
                    ct::String("linked animation '") + file.animationLinks[i] + "': " + error);
    }

    out.materials.resize(file.materials.size());
    for (std::size_t i = 0; i < file.materials.size(); ++i)
        out.materials[i].name = file.materials[i];

    out.meshes.resize(file.meshes.size());
    for (std::size_t m = 0; m < file.meshes.size(); ++m)
    {
        ModelMeshData& mesh = out.meshes[m];
        mesh.name = file.meshes[m].name;
        for (std::size_t i = 0; i < file.meshes[m].subMeshes.size(); ++i)
        {
            const SubMeshRecord& record = file.meshes[m].subMeshes[i];
            const SurfaceRef& surface = owned->surfaces[record.surface];
            ModelPrimitiveData primitive;
            primitive.topology = static_cast<prisma::Topology>(surface.topology);
            primitive.material =
                    record.materialSlot == kNoSlot ? -1 : static_cast<int>(record.materialSlot);
            std::uint32_t mask = kVertexPosition;
            if (surface.attribs & HasNormal) mask |= kVertexNormal;
            if (surface.attribs & HasTangent) mask |= kVertexTangent;
            if (surface.attribs & HasUV) mask |= kVertexUv;
            if (surface.attribs & HasUV2) mask |= kVertexUv2;
            if (surface.attribs & HasColor) mask |= kVertexColor;
            if (surface.hasSkin) mask |= kVertexJoints | kVertexWeights;
            primitive.attributeMask = mask;
            primitive.vertexCount = surface.vertexCount;
            primitive.indexCount = surface.hasIndices ? surface.indexCount : 0;
            primitive.sourceBytes = surfaceBytes(surface);
            primitive.source = record.surface;
            primitive.aabb = Math::Box(
                    Math::Vec3(record.boundsMin[0], record.boundsMin[1], record.boundsMin[2]),
                    Math::Vec3(record.boundsMax[0], record.boundsMax[1], record.boundsMax[2]));
            primitive.hasAabb = (record.flags & SubMeshHasBounds) != 0;
            primitive.skinned = (record.flags & SubMeshSkinned) != 0;
            out.geometryBytes += primitive.sourceBytes;
            ++out.primitiveCount;
            mesh.primitives.push_back(primitive);
        }
    }
    for (std::size_t i = 0; i < file.shadows.size(); ++i)
        out.meshes[file.shadows[i].mesh].shadowSurfaces =
                static_cast<ct::Vector<SurfaceData>&&>(file.shadows[i].surfaces);

    out.nodes = static_cast<ct::Vector<ModelNodeData>&&>(file.nodes);
    out.roots = static_cast<ct::Vector<int>&&>(file.roots);
    out.cameras = static_cast<ct::Vector<ModelCameraData>&&>(file.cameras);

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<PMeshSource>&&>(owned));
    return true;
}

bool readVec3Array(Reader& r, ct::Vector<Math::Vec3>& out, std::uint32_t count)
{
    if (count > kMaxKeys || !r.fits(count, 12)) return r.fail("invalid key count");
    out.resize(count);
    return count == 0 || r.floats(&out[0].x, static_cast<std::size_t>(count) * 3);
}

bool readQuatArray(Reader& r, ct::Vector<Math::Quaternion>& out, std::uint32_t count)
{
    if (count > kMaxKeys || !r.fits(count, 16)) return r.fail("invalid key count");
    out.resize(count);
    return count == 0 || r.floats(&out[0].x, static_cast<std::size_t>(count) * 4);
}

bool readFloatArray(Reader& r, ct::Vector<float>& out, std::uint32_t count)
{
    if (count > kMaxKeys || !r.fits(count, 4)) return r.fail("invalid key count");
    out.resize(count);
    return count == 0 || r.floats(out.data(), count);
}

bool readCount(Reader& r, std::uint32_t& count, std::uint64_t record)
{
    if (!r.u32(count)) return false;
    if (count > kMaxTracks || !r.fits(count, record)) return r.fail("invalid track count");
    return true;
}

bool readStringArray(Reader& r, ct::Vector<ct::String>& out, std::uint32_t count)
{
    if (count > kMaxKeys || !r.fits(count, 4)) return r.fail("invalid key count");
    out.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
        if (!r.string(out[i])) return false;
    return true;
}

bool readClip(Reader& r, const ct::Vector<Skeleton>& skeletons, AnimationClip& clip)
{
    ct::String name;
    float duration = 0.0f;
    std::int32_t skeleton = 0;
    std::uint32_t loop = 0;
    std::uint32_t counts[6] = { 0, 0, 0, 0, 0, 0 };
    if (!r.string(name) || !r.f32Finite(duration) || !r.s32(skeleton) || !r.u32(loop)) return false;
    if (duration < 0.0f) return r.fail("negative clip duration");
    if (loop >= kLoopModeCount) return r.fail("invalid loop mode");
    const std::uint64_t records[6] = { 24, 28, 12, 16, 12, 8 };
    for (int i = 0; i < 6; ++i)
        if (!readCount(r, counts[i], records[i])) return false;

    clip.setName(name);
    clip.setDuration(duration);
    clip.setSkeleton(skeleton);
    clip.setLoopMode(static_cast<LoopMode>(loop));
    const Skeleton* bones = skeleton >= 0 && static_cast<std::size_t>(skeleton) < skeletons.size()
                                    ? &skeletons[static_cast<std::size_t>(skeleton)]
                                    : nullptr;

    clip.tracks().resize(counts[0]);
    for (std::uint32_t i = 0; i < counts[0]; ++i)
    {
        BoneTrack& track = clip.tracks()[i];
        std::int32_t bone = 0;
        ct::String boneName;
        std::uint32_t n[4] = { 0, 0, 0, 0 };
        if (!r.s32(bone) || !r.string(boneName)) return false;
        if (bone < -1) return r.fail("invalid bone reference");
        if (bone >= 0)
        {
            if (!bones) return r.fail("bone track in a clip without a skeleton");
            const bool direct = static_cast<std::uint32_t>(bone) < bones->boneCount() &&
                                bones->bone(static_cast<std::uint32_t>(bone)).name == boneName;
            if (!direct)
            {
                bone = bones->findBone(boneName.c_str());
                if (bone < 0)
                    return r.fail("bone track refers to a bone that is not in the skeleton");
            }
        }
        track.bone = bone;
        if (!r.u32(n[0]) || !r.u32(n[1]) || !r.u32(n[2]) || !r.u32(n[3])) return false;
        if (!readFloatArray(r, track.times, n[0]) || !readVec3Array(r, track.positions, n[1]) ||
                !readQuatArray(r, track.rotations, n[2]) || !readVec3Array(r, track.scales, n[3]))
            return false;
    }

    clip.nodeTracks().resize(counts[1]);
    for (std::uint32_t i = 0; i < counts[1]; ++i)
    {
        NodeTrack& track = clip.nodeTracks()[i];
        std::uint32_t n[6] = { 0, 0, 0, 0, 0, 0 };
        if (!r.string(track.path)) return false;
        for (int k = 0; k < 6; ++k)
            if (!r.u32(n[k])) return false;
        if (!readFloatArray(r, track.positionTimes, n[0]) ||
                !readVec3Array(r, track.positions, n[1]) ||
                !readFloatArray(r, track.rotationTimes, n[2]) ||
                !readQuatArray(r, track.rotations, n[3]) ||
                !readFloatArray(r, track.scaleTimes, n[4]) || !readVec3Array(r, track.scales, n[5]))
            return false;
    }

    clip.methodTracks().resize(counts[2]);
    for (std::uint32_t i = 0; i < counts[2]; ++i)
    {
        MethodTrack& track = clip.methodTracks()[i];
        std::uint32_t n[2] = { 0, 0 };
        if (!r.string(track.path) || !r.u32(n[0]) || !r.u32(n[1])) return false;
        if (!readFloatArray(r, track.times, n[0]) || !readStringArray(r, track.methods, n[1]))
            return false;
    }

    clip.valueTracks().resize(counts[3]);
    for (std::uint32_t i = 0; i < counts[3]; ++i)
    {
        ValueTrack& track = clip.valueTracks()[i];
        std::uint32_t n[2] = { 0, 0 };
        if (!r.string(track.path) || !r.string(track.property) || !r.u32(n[0]) || !r.u32(n[1]))
            return false;
        if (!readFloatArray(r, track.times, n[0]) || !readFloatArray(r, track.values, n[1]))
            return false;
    }

    clip.animationTracks().resize(counts[4]);
    for (std::uint32_t i = 0; i < counts[4]; ++i)
    {
        AnimationTrack& track = clip.animationTracks()[i];
        std::uint32_t n[2] = { 0, 0 };
        if (!r.string(track.path) || !r.u32(n[0]) || !r.u32(n[1])) return false;
        if (!readFloatArray(r, track.times, n[0]) || !readStringArray(r, track.animations, n[1]))
            return false;
    }

    for (std::uint32_t i = 0; i < counts[5]; ++i)
    {
        float time = 0.0f;
        ct::String eventName;
        if (!r.f32Finite(time) || !r.string(eventName)) return false;
        clip.addEvent(time, eventName);
    }
    if (clip.events().size() != counts[5]) return r.fail("invalid animation events");
    return true;
}

} // namespace

bool readPMeshSkeleton(const std::uint8_t* data, std::size_t size, Skeleton& out, ct::String& error)
{
    Reader r(data, size);
    std::uint32_t flags = 0;
    bool ok = r.header(kSkeletonMagic, flags);
    bool found = false;
    bool sawEnd = false;
    Skeleton loaded;
    while (ok)
    {
        if (r.remaining() == 0)
        {
            ok = r.fail("missing end chunk, the file is truncated");
            break;
        }
        std::uint32_t id = 0;
        std::uint64_t chunkSize = 0;
        if (!r.next(id, chunkSize) || !r.enter(chunkSize))
        {
            ok = false;
            break;
        }
        if (id == Bones)
        {
            std::uint32_t count = 0;
            ok = !found || r.fail("duplicate bone chunk");
            ok = ok && r.u32(count);
            if (ok && (count > kMaxBones || !r.fits(count, kBoneFixedSize)))
                ok = r.fail("invalid bone count");
            for (std::uint32_t i = 0; ok && i < count; ++i)
            {
                ct::String boneName;
                std::int32_t parent = -1;
                Math::Mat4 local;
                Math::Mat4 inverse;
                ok = r.string(boneName) && r.s32(parent) && r.mat4(local) && r.mat4(inverse);
                if (ok && !loaded.addBone(boneName, parent, local, inverse))
                    ok = r.fail("invalid bone");
            }
            found = true;
        }
        else if (id == End)
            sawEnd = true;
        r.leave();
        if (ok && sawEnd) break;
    }
    if (ok && !found) ok = r.fail("no bone chunk");
    if (ok && r.remaining() != 0) ok = r.fail("data after the end chunk");
    if (ok && !loaded.finalize()) ok = r.fail("bone hierarchy is invalid");
    if (!ok)
    {
        error = r.error();
        return false;
    }
    out = static_cast<Skeleton&&>(loaded);
    return true;
}

bool readPMeshAnimations(const std::uint8_t* data, std::size_t size,
        const ct::Vector<Skeleton>& skeletons, ct::Vector<AnimationClip>& out, ct::String& error)
{
    Reader r(data, size);
    std::uint32_t flags = 0;
    bool ok = r.header(kAnimationMagic, flags);
    bool sawEnd = false;
    ct::Vector<AnimationClip> loaded;
    while (ok)
    {
        if (r.remaining() == 0)
        {
            ok = r.fail("missing end chunk, the file is truncated");
            break;
        }
        std::uint32_t id = 0;
        std::uint64_t chunkSize = 0;
        if (!r.next(id, chunkSize) || !r.enter(chunkSize))
        {
            ok = false;
            break;
        }
        if (id == Clip)
        {
            ok = loaded.size() < kMaxClips || r.fail("too many clips");
            if (ok)
            {
                loaded.push_back(AnimationClip());
                ok = readClip(r, skeletons, loaded.back());
            }
        }
        else if (id == End)
            sawEnd = true;
        r.leave();
        if (ok && sawEnd) break;
    }
    if (ok && r.remaining() != 0) ok = r.fail("data after the end chunk");
    if (!ok)
    {
        error = r.error();
        return false;
    }
    for (std::size_t i = 0; i < loaded.size(); ++i)
        out.push_back(static_cast<AnimationClip&&>(loaded[i]));
    return true;
}

bool readPMeshLinks(const std::uint8_t* data, std::size_t size, ct::Vector<ct::String>& skeletons,
        ct::Vector<ct::String>& animations, ct::String& error)
{
    ct::Vector<SurfaceRef> surfaces;
    ParsedFile file;
    if (!parseMesh(data, size, surfaces, file, error)) return false;
    skeletons = file.skeletonLinks;
    animations = file.animationLinks;
    return true;
}

const char* PMeshImporter::name() const
{
    return "pmesh";
}

bool PMeshImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, kMeshExtension) == 0;
}

bool PMeshImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(static_cast<ct::Vector<std::uint8_t>&&>(bytes), path, out);
}

bool PMeshImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty mesh data");
    ct::Vector<std::uint8_t> bytes;
    bytes.resize(size);
    memcpy(bytes.data(), data, size);
    return build(static_cast<ct::Vector<std::uint8_t>&&>(bytes), virtualName, out);
}

bool PMeshImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const PMeshSource* source = static_cast<const PMeshSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    const SurfaceRef& surface = source->surfaces[primitive.source];
    out = SurfaceData();
    out.topology = static_cast<prisma::Topology>(surface.topology);

    const std::uint32_t count = surface.vertexCount;
    const std::uint32_t stride = vertexAttribStride(surface.attribs);
    const std::uint8_t* base = source->bytes.data();
    Reader vertices(base + surface.vertexOffset, static_cast<std::size_t>(count) * stride);
    out.positions.resize(count);
    if (surface.attribs & HasNormal) out.normals.resize(count);
    if (surface.attribs & HasTangent) out.tangents.resize(count);
    if (surface.attribs & HasUV) out.uvs.resize(count);
    if (surface.attribs & HasUV2) out.uvs2.resize(count);
    if (surface.attribs & HasColor) out.colors.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        bool ok = vertices.floats(&out.positions[i].x, 3);
        if (surface.attribs & HasNormal) ok = ok && vertices.floats(&out.normals[i].x, 3);
        if (surface.attribs & HasTangent) ok = ok && vertices.floats(&out.tangents[i].x, 4);
        if (surface.attribs & HasUV) ok = ok && vertices.floats(&out.uvs[i].x, 2);
        if (surface.attribs & HasUV2) ok = ok && vertices.floats(&out.uvs2[i].x, 2);
        if (surface.attribs & HasColor) ok = ok && vertices.floats(&out.colors[i].x, 4);
        if (!ok) return false;
    }

    if (surface.hasSkin)
    {
        Reader skin(base + surface.skinOffset, static_cast<std::size_t>(count) * kSkinVertexSize);
        out.joints.resize(count);
        out.weights.resize(count);
        for (std::uint32_t i = 0; i < count; ++i)
            if (!skin.u32(out.joints[i]) || !skin.floats(&out.weights[i].x, 4)) return false;
    }

    if (surface.hasIndices)
    {
        const std::size_t width = surface.index16 ? 2 : 4;
        Reader indices(base + surface.indexOffset,
                static_cast<std::size_t>(surface.indexCount) * width);
        out.indices.resize(surface.indexCount);
        if (surface.index16)
        {
            for (std::uint32_t i = 0; i < surface.indexCount; ++i)
            {
                std::uint16_t value = 0;
                if (!indices.u16(value)) return false;
                out.indices[i] = value;
            }
        }
        else if (!indices.words(out.indices.data(), surface.indexCount))
            return false;
        for (std::uint32_t i = 0; i < surface.indexCount; ++i)
            if (out.indices[i] >= count) return false;
    }
    return true;
}

bool PMeshImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
