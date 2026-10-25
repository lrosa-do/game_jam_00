#include "prisma3d/import/OgreMeshImporter.h"

#include "prisma3d/core/FileSystem.h"

#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint16_t M_HEADER = 0x1000;
const std::uint16_t M_MESH = 0x3000;
const std::uint16_t M_SUBMESH = 0x4000;
const std::uint16_t M_SUBMESH_OPERATION = 0x4010;
const std::uint16_t M_SUBMESH_BONE_ASSIGNMENT = 0x4100;
const std::uint16_t M_SUBMESH_TEXTURE_ALIAS = 0x4200;
const std::uint16_t M_GEOMETRY = 0x5000;
const std::uint16_t M_GEOMETRY_VERTEX_DECLARATION = 0x5100;
const std::uint16_t M_GEOMETRY_VERTEX_ELEMENT = 0x5110;
const std::uint16_t M_GEOMETRY_VERTEX_BUFFER = 0x5200;
const std::uint16_t M_GEOMETRY_VERTEX_BUFFER_DATA = 0x5210;
const std::uint16_t M_MESH_SKELETON_LINK = 0x6000;
const std::uint16_t M_MESH_BONE_ASSIGNMENT = 0x7000;
const std::uint16_t M_MESH_LOD_LEVEL = 0x8000;
const std::uint16_t M_MESH_LOD_USAGE = 0x8100;
const std::uint16_t M_MESH_LOD_MANUAL = 0x8110;
const std::uint16_t M_MESH_LOD_GENERATED = 0x8120;

const std::uint16_t SKELETON_HEADER = 0x1000;
const std::uint16_t SKELETON_BLENDMODE = 0x1010;
const std::uint16_t SKELETON_BONE = 0x2000;
const std::uint16_t SKELETON_BONE_PARENT = 0x3000;
const std::uint16_t SKELETON_ANIMATION = 0x4000;
const std::uint16_t SKELETON_ANIMATION_BASEINFO = 0x4010;
const std::uint16_t SKELETON_ANIMATION_TRACK = 0x4100;
const std::uint16_t SKELETON_ANIMATION_TRACK_KEYFRAME = 0x4110;
const std::uint16_t SKELETON_ANIMATION_LINK = 0x5000;

const std::size_t kChunkHeaderSize = 6;
const std::size_t kMaxStringLength = 4096;
const std::uint32_t kMaxVertices = 16u * 1024u * 1024u;
const std::uint32_t kMaxIndices = 64u * 1024u * 1024u;
const std::size_t kMaxBoneHandles = 256;

const std::uint16_t VES_POSITION = 1;
const std::uint16_t VES_NORMAL = 4;
const std::uint16_t VES_DIFFUSE = 5;
const std::uint16_t VES_TEXTURE_COORDINATES = 7;
const std::uint16_t VES_TANGENT = 9;

const std::uint16_t VET_FLOAT1 = 0;
const std::uint16_t VET_FLOAT2 = 1;
const std::uint16_t VET_FLOAT3 = 2;
const std::uint16_t VET_FLOAT4 = 3;
const std::uint16_t VET_COLOUR = 4;
const std::uint16_t VET_COLOUR_ARGB = 10;
const std::uint16_t VET_COLOUR_ABGR = 11;

const std::uint16_t OT_TRIANGLE_LIST = 4;
const std::uint16_t OT_TRIANGLE_STRIP = 5;
const std::uint16_t OT_TRIANGLE_FAN = 6;

bool finite(float value)
{
    return value - value == 0.0f;
}

struct Cursor
{
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::size_t pos = 0;
    const char* error = nullptr;

    bool fail(const char* message)
    {
        if (!error) error = message;
        return false;
    }

    bool canRead(std::size_t count) const
    {
        return !error && pos <= size && size - pos >= count;
    }

    bool take(void* out, std::size_t count)
    {
        if (!canRead(count))
        {
            pos = size;
            return fail("data is truncated");
        }
        memcpy(out, data + pos, count);
        pos += count;
        return true;
    }

    std::uint8_t readU8()
    {
        std::uint8_t value = 0;
        take(&value, 1);
        return value;
    }

    bool readBool()
    {
        return readU8() != 0;
    }

    std::uint16_t readU16()
    {
        std::uint8_t raw[2] = { 0, 0 };
        take(raw, 2);
        return static_cast<std::uint16_t>(raw[0] | (raw[1] << 8));
    }

    std::uint32_t readU32()
    {
        std::uint8_t raw[4] = { 0, 0, 0, 0 };
        take(raw, 4);
        return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8) |
               (static_cast<std::uint32_t>(raw[2]) << 16) |
               (static_cast<std::uint32_t>(raw[3]) << 24);
    }

    float readF32()
    {
        const std::uint32_t raw = readU32();
        float value;
        memcpy(&value, &raw, sizeof(value));
        return value;
    }

    void seek(std::size_t target)
    {
        pos = target > size ? size : target;
    }
};

struct ChunkHeader
{
    std::uint16_t id = 0;
    std::uint32_t length = 0;
};

bool readChunkHeader(Cursor& r, std::size_t parentEnd, ChunkHeader& header, std::size_t& childEnd)
{
    if (r.error) return false;
    if (parentEnd > r.size || r.pos > parentEnd || parentEnd - r.pos < kChunkHeaderSize)
        return r.fail("chunk header is truncated");
    header.id = r.readU16();
    header.length = r.readU32();
    if (header.length < kChunkHeaderSize) return r.fail("chunk length is invalid");
    const std::size_t payload = static_cast<std::size_t>(header.length) - kChunkHeaderSize;
    if (payload > parentEnd - r.pos) return r.fail("chunk exceeds its parent");
    childEnd = r.pos + payload;
    return true;
}

bool readChunkHeaderLoose(Cursor& r, std::size_t parentEnd, ChunkHeader& header,
        std::size_t& childEnd)
{
    if (r.error) return false;
    if (parentEnd > r.size || r.pos > parentEnd || parentEnd - r.pos < kChunkHeaderSize)
        return r.fail("chunk header is truncated");
    header.id = r.readU16();
    header.length = r.readU32();
    if (header.length < kChunkHeaderSize) return r.fail("chunk length is invalid");
    const std::size_t payload = static_cast<std::size_t>(header.length) - kChunkHeaderSize;
    childEnd = payload > parentEnd - r.pos ? parentEnd : r.pos + payload;
    return true;
}

bool readOgreString(Cursor& r, std::size_t endPos, ct::String& value)
{
    if (r.error) return false;
    if (endPos > r.size || r.pos > endPos) return r.fail("string is out of range");
    const std::size_t available = endPos - r.pos;
    const std::size_t scanLength =
            available < kMaxStringLength + 1 ? available : kMaxStringLength + 1;
    const std::uint8_t* begin = r.data + r.pos;
    const void* terminator = memchr(begin, '\n', scanLength);
    if (!terminator) return r.fail("string is not terminated");
    const std::size_t length =
            static_cast<std::size_t>(static_cast<const std::uint8_t*>(terminator) - begin);
    value = ct::String(reinterpret_cast<const char*>(begin), length);
    r.pos += length + 1;
    return true;
}

std::size_t vertexElementSize(std::uint16_t type)
{
    switch (type)
    {
        case VET_FLOAT1:
            return sizeof(float);
        case VET_FLOAT2:
            return sizeof(float) * 2;
        case VET_FLOAT3:
            return sizeof(float) * 3;
        case VET_FLOAT4:
            return sizeof(float) * 4;
        case VET_COLOUR:
        case VET_COLOUR_ARGB:
        case VET_COLOUR_ABGR:
            return sizeof(std::uint32_t);
        default:
            return 0;
    }
}

bool readVec3(Cursor& r, std::size_t end, Math::Vec3& value)
{
    if (end > r.size || r.pos > end || end - r.pos < sizeof(float) * 3)
        return r.fail("vector is truncated");
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    return true;
}

bool readQuat(Cursor& r, std::size_t end, Math::Quaternion& value)
{
    if (end > r.size || r.pos > end || end - r.pos < sizeof(float) * 4)
        return r.fail("quaternion is truncated");
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    value.w = r.readF32();
    const float length2 = value.LengthSquared();
    if (!finite(length2) || length2 <= 1e-12f) return r.fail("quaternion is invalid");
    value = value.Normalized();
    return true;
}

LocalPose localPose(const Math::Vec3& position, const Math::Quaternion& rotation,
        const Math::Vec3& scale)
{
    LocalPose pose;
    pose.position = position;
    pose.rotation = rotation;
    pose.scale = scale;
    return pose;
}

struct BoneAssignment
{
    std::uint32_t vertex = 0;
    std::uint16_t bone = 0;
    float weight = 0.0f;
};

bool readBoneAssignment(Cursor& r, std::size_t end, BoneAssignment& assignment)
{
    if (end < r.pos || end - r.pos != sizeof(std::uint32_t) + sizeof(std::uint16_t) + sizeof(float))
        return r.fail("bone assignment has an invalid size");
    assignment.vertex = r.readU32();
    assignment.bone = r.readU16();
    assignment.weight = r.readF32();
    if (assignment.bone > 255 || !finite(assignment.weight) || assignment.weight < 0.0f)
        return r.fail("bone assignment is invalid");
    return true;
}

void applyAssignments(SurfaceData& surface, const ct::Vector<BoneAssignment>& assignments)
{
    const std::size_t vertexCount = surface.positions.size();
    surface.joints.clear();
    surface.weights.clear();
    surface.joints.resize(vertexCount, 0u);
    surface.weights.resize(vertexCount, Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));

    ct::Vector<std::uint32_t> start;
    start.resize(vertexCount + 1, 0u);
    for (std::size_t i = 0; i < assignments.size(); ++i)
    {
        const BoneAssignment& assignment = assignments[i];
        if (assignment.vertex < vertexCount && assignment.weight > 0.0f)
            ++start[assignment.vertex + 1];
    }
    for (std::size_t v = 0; v < vertexCount; ++v) start[v + 1] += start[v];

    ct::Vector<std::uint32_t> cursor = start;
    ct::Vector<BoneAssignment> sorted;
    sorted.resize(start[vertexCount]);
    for (std::size_t i = 0; i < assignments.size(); ++i)
    {
        const BoneAssignment& assignment = assignments[i];
        if (assignment.vertex < vertexCount && assignment.weight > 0.0f)
            sorted[cursor[assignment.vertex]++] = assignment;
    }

    for (std::size_t v = 0; v < vertexCount; ++v)
    {
        const std::size_t first = start[v];
        const std::size_t last = start[v + 1];
        for (std::size_t i = first + 1; i < last; ++i)
        {
            const BoneAssignment current = sorted[i];
            std::size_t k = i;
            while (k > first && sorted[k - 1].weight < current.weight)
            {
                sorted[k] = sorted[k - 1];
                --k;
            }
            sorted[k] = current;
        }
        const std::size_t available = last - first;
        const std::size_t count = available < 4 ? available : 4;
        float total = 0.0f;
        for (std::size_t i = 0; i < count; ++i) total += sorted[first + i].weight;
        if (total <= 1e-8f) continue;
        std::uint32_t joints = 0;
        float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        for (std::size_t i = 0; i < count; ++i)
        {
            joints |= static_cast<std::uint32_t>(sorted[first + i].bone) << (8 * i);
            weights[i] = sorted[first + i].weight / total;
        }
        surface.joints[v] = joints;
        surface.weights[v] = Math::Vec4(weights[0], weights[1], weights[2], weights[3]);
    }
}

struct OgreBoneData
{
    ct::String name;
    std::uint16_t handle = 0;
    int parent = -1;
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion rotation = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
};

struct PendingTrack
{
    std::uint16_t handle = 0;
    ct::Vector<float> times;
    ct::Vector<Math::Vec3> translations;
    ct::Vector<Math::Quaternion> rotations;
    ct::Vector<Math::Vec3> scales;
};

struct PendingAnimation
{
    ct::String name;
    float duration = 0.0f;
    ct::Vector<PendingTrack> tracks;
};

struct ParentLink
{
    std::uint16_t child = 0;
    std::uint16_t parent = 0;
};

struct SkeletonResult
{
    Skeleton skeleton;
    ct::Vector<AnimationClip> clips;
    int handleToBone[kMaxBoneHandles];

    SkeletonResult()
    {
        for (std::size_t i = 0; i < kMaxBoneHandles; ++i) handleToBone[i] = -1;
    }
};

bool readSkeletonAnimation(Cursor& r, std::size_t animationEnd, PendingAnimation& animation)
{
    if (!readOgreString(r, animationEnd, animation.name) || animationEnd - r.pos < sizeof(float))
        return r.fail("animation is truncated");
    animation.duration = r.readF32();
    if (!finite(animation.duration) || animation.duration < 0.0f)
        return r.fail("animation duration is invalid");

    while (r.pos < animationEnd)
    {
        ChunkHeader chunk;
        std::size_t chunkEnd = 0;
        if (!readChunkHeader(r, animationEnd, chunk, chunkEnd)) return false;
        if (chunk.id == SKELETON_ANIMATION_BASEINFO)
        {
            ct::String baseName;
            if (!readOgreString(r, chunkEnd, baseName) || chunkEnd - r.pos != sizeof(float))
                return r.fail("animation base info is invalid");
            r.readF32();
        }
        else if (chunk.id == SKELETON_ANIMATION_TRACK)
        {
            if (chunkEnd - r.pos < sizeof(std::uint16_t)) return r.fail("track is truncated");
            PendingTrack track;
            track.handle = r.readU16();
            while (r.pos < chunkEnd)
            {
                ChunkHeader keyChunk;
                std::size_t keyEnd = 0;
                if (!readChunkHeader(r, chunkEnd, keyChunk, keyEnd)) return false;
                if (keyChunk.id != SKELETON_ANIMATION_TRACK_KEYFRAME)
                    return r.fail("track holds an unexpected chunk");
                const std::size_t required = sizeof(float) * 8;
                const std::size_t payload = keyEnd - r.pos;
                if (payload != required && payload != required + sizeof(float) * 3)
                    return r.fail("keyframe has an invalid size");
                const float time = r.readF32();
                Math::Quaternion rotation;
                Math::Vec3 translation;
                Math::Vec3 scale(1.0f, 1.0f, 1.0f);
                if (!finite(time) || !readQuat(r, keyEnd, rotation) ||
                        !readVec3(r, keyEnd, translation))
                    return r.fail("keyframe is invalid");
                if (r.pos < keyEnd && !readVec3(r, keyEnd, scale)) return false;
                if (!track.times.empty() && time < track.times.back())
                    return r.fail("keyframes are not in time order");
                track.times.push_back(time);
                track.rotations.push_back(rotation);
                track.translations.push_back(translation);
                track.scales.push_back(scale);
            }
            animation.tracks.push_back(static_cast<PendingTrack&&>(track));
        }
        else
            r.seek(chunkEnd);
    }
    return !r.error;
}

bool buildGlobal(const ct::Vector<OgreBoneData>& bones, ct::Vector<Math::Mat4>& globals,
        ct::Vector<std::uint8_t>& state, std::size_t index)
{
    if (state[index] == 2) return true;
    if (state[index] == 1) return false;
    state[index] = 1;
    const int parent = bones[index].parent;
    if (parent >= 0 && !buildGlobal(bones, globals, state, static_cast<std::size_t>(parent)))
        return false;
    const Math::Mat4 local =
            localPose(bones[index].position, bones[index].rotation, bones[index].scale).toMatrix();
    globals[index] = parent >= 0 ? globals[static_cast<std::size_t>(parent)] * local : local;
    state[index] = 2;
    return true;
}

bool loadSkeleton(const ct::Vector<std::uint8_t>& bytes, SkeletonResult& result, const char*& error)
{
    Cursor r;
    r.data = bytes.data();
    r.size = bytes.size();

    const bool parsed = [&]()
    {
        if (!r.canRead(sizeof(std::uint16_t)) || r.readU16() != SKELETON_HEADER)
            return r.fail("not an Ogre skeleton file");
        ct::String version;
        if (!readOgreString(r, r.size, version)) return false;
        if (version != "[Serializer_v1.10]" && version != "[Serializer_v1.80]")
            return r.fail("unsupported skeleton version");

        ct::Vector<OgreBoneData> bones;
        int tempByHandle[kMaxBoneHandles];
        for (std::size_t i = 0; i < kMaxBoneHandles; ++i) tempByHandle[i] = -1;
        ct::Vector<ParentLink> parents;
        ct::Vector<PendingAnimation> animations;

        while (r.pos < r.size)
        {
            ChunkHeader chunk;
            std::size_t chunkEnd = 0;
            if (!readChunkHeader(r, r.size, chunk, chunkEnd)) return false;
            if (chunk.id == SKELETON_BLENDMODE)
            {
                if (chunkEnd - r.pos != sizeof(std::uint16_t))
                    return r.fail("blend mode chunk is invalid");
                r.readU16();
            }
            else if (chunk.id == SKELETON_BONE)
            {
                OgreBoneData bone;
                if (!readOgreString(r, r.size, bone.name)) return false;
                if (bone.name.size() + 1 > r.size - chunkEnd) return r.fail("bone is truncated");
                chunkEnd += bone.name.size() + 1;
                if (chunkEnd < r.pos || (chunkEnd - r.pos != 30 && chunkEnd - r.pos != 42))
                    return r.fail("bone chunk has an invalid size");
                bone.handle = r.readU16();
                if (bone.handle > 255 || tempByHandle[bone.handle] >= 0)
                    return r.fail("bone handle is invalid or repeated");
                if (!readVec3(r, chunkEnd, bone.position) || !readQuat(r, chunkEnd, bone.rotation))
                    return false;
                if (r.pos < chunkEnd && !readVec3(r, chunkEnd, bone.scale)) return false;
                tempByHandle[bone.handle] = static_cast<int>(bones.size());
                bones.push_back(static_cast<OgreBoneData&&>(bone));
            }
            else if (chunk.id == SKELETON_BONE_PARENT)
            {
                if (chunkEnd - r.pos != sizeof(std::uint16_t) * 2)
                    return r.fail("bone parent chunk is invalid");
                ParentLink link;
                link.child = r.readU16();
                link.parent = r.readU16();
                parents.push_back(link);
            }
            else if (chunk.id == SKELETON_ANIMATION)
            {
                PendingAnimation animation;
                if (!readSkeletonAnimation(r, chunkEnd, animation)) return false;
                animations.push_back(static_cast<PendingAnimation&&>(animation));
            }
            else if (chunk.id == SKELETON_ANIMATION_LINK)
                return r.fail("linked animation skeletons are not supported");
            else
                r.seek(chunkEnd);
        }
        if (r.error) return false;

        for (std::size_t i = 0; i < parents.size(); ++i)
        {
            const int child =
                    parents[i].child < kMaxBoneHandles ? tempByHandle[parents[i].child] : -1;
            const int parent =
                    parents[i].parent < kMaxBoneHandles ? tempByHandle[parents[i].parent] : -1;
            if (child < 0 || parent < 0 || child == parent)
                return r.fail("bone parent link is invalid");
            bones[static_cast<std::size_t>(child)].parent = parent;
        }

        ct::Vector<Math::Mat4> globals;
        globals.resize(bones.size(), Math::Mat4::Identity());
        ct::Vector<std::uint8_t> state;
        state.resize(bones.size(), static_cast<std::uint8_t>(0));
        for (std::size_t i = 0; i < bones.size(); ++i)
        {
            if (!buildGlobal(bones, globals, state, i)) return r.fail("bone hierarchy has a cycle");
        }

        for (std::size_t i = 0; i < bones.size(); ++i)
        {
            result.handleToBone[bones[i].handle] = static_cast<int>(i);
            const Math::Mat4 bindLocal =
                    localPose(bones[i].position, bones[i].rotation, bones[i].scale).toMatrix();
            if (!result.skeleton.addBone(bones[i].name, bones[i].parent, bindLocal,
                        globals[i].Inverse()))
                return r.fail("bone cannot be added to the skeleton");
        }
        if (!result.skeleton.finalize()) return r.fail("skeleton cannot be finalized");

        for (std::size_t a = 0; a < animations.size(); ++a)
        {
            PendingAnimation& source = animations[a];
            AnimationClip clip;
            clip.setName(source.name);
            clip.setDuration(source.duration);
            clip.setSkeleton(0);
            for (std::size_t t = 0; t < source.tracks.size(); ++t)
            {
                PendingTrack& sourceTrack = source.tracks[t];
                const int mapped = sourceTrack.handle < kMaxBoneHandles
                                           ? tempByHandle[sourceTrack.handle]
                                           : -1;
                if (mapped < 0) return r.fail("animation track references a missing bone");
                const OgreBoneData& bind = bones[static_cast<std::size_t>(mapped)];
                BoneTrack track;
                track.bone = mapped;
                track.times = static_cast<ct::Vector<float>&&>(sourceTrack.times);
                for (std::size_t key = 0; key < track.times.size(); ++key)
                {
                    track.positions.push_back(bind.position + sourceTrack.translations[key]);
                    Math::Quaternion rotation =
                            (bind.rotation * sourceTrack.rotations[key]).Normalized();
                    if (!track.rotations.empty() && track.rotations.back().Dot(rotation) < 0.0f)
                        rotation = -rotation;
                    track.rotations.push_back(rotation);
                    track.scales.push_back(Math::Vec3(bind.scale.x * sourceTrack.scales[key].x,
                            bind.scale.y * sourceTrack.scales[key].y,
                            bind.scale.z * sourceTrack.scales[key].z));
                }
                clip.tracks().push_back(static_cast<BoneTrack&&>(track));
            }
            result.clips.push_back(static_cast<AnimationClip&&>(clip));
        }
        return true;
    }();

    if (!parsed)
    {
        error = r.error ? r.error : "skeleton is invalid";
        return false;
    }
    return true;
}

struct VertexElementDecl
{
    std::uint16_t source = 0;
    std::uint16_t type = 0;
    std::uint16_t semantic = 0;
    std::uint16_t offset = 0;
    std::uint16_t index = 0;
};

struct VertexBufferBind
{
    std::uint16_t source = 0;
    std::uint16_t vertexSize = 0;
    ct::Vector<std::uint8_t> data;
};

struct GeometryLayout
{
    bool hasNormal = false;
    bool hasTangent = false;
    bool hasColor = false;
    bool hasUv = false;
    bool hasUv2 = false;
};

const VertexBufferBind* findBuffer(const ct::Vector<VertexBufferBind>& buffers,
        const VertexElementDecl* element)
{
    if (!element) return nullptr;
    for (std::size_t i = 0; i < buffers.size(); ++i)
    {
        if (buffers[i].source == element->source) return &buffers[i];
    }
    return nullptr;
}

const std::uint8_t* elementAt(const VertexBufferBind* buffer, const VertexElementDecl* element,
        std::size_t vertex)
{
    return buffer->data.data() + vertex * buffer->vertexSize + element->offset;
}

bool readGeometry(Cursor& r, std::size_t endPos, SurfaceData& surface, GeometryLayout& layout)
{
    if (endPos > r.size || r.pos > endPos || endPos - r.pos < sizeof(std::uint32_t))
        return r.fail("geometry is truncated");
    const std::uint32_t vertexCount = r.readU32();
    if (vertexCount == 0 || vertexCount > kMaxVertices)
        return r.fail("geometry vertex count is invalid");

    ct::Vector<VertexElementDecl> decl;
    ct::Vector<VertexBufferBind> buffers;

    while (r.pos < endPos)
    {
        ChunkHeader header;
        std::size_t childEnd = 0;
        if (!readChunkHeader(r, endPos, header, childEnd)) return false;

        if (header.id == M_GEOMETRY_VERTEX_DECLARATION)
        {
            while (r.pos < childEnd)
            {
                ChunkHeader elementHeader;
                std::size_t elementEnd = 0;
                if (!readChunkHeader(r, childEnd, elementHeader, elementEnd)) return false;
                if (elementHeader.id != M_GEOMETRY_VERTEX_ELEMENT || elementEnd - r.pos != 10)
                    return r.fail("vertex element is invalid");
                VertexElementDecl element;
                element.source = r.readU16();
                element.type = r.readU16();
                element.semantic = r.readU16();
                element.offset = r.readU16();
                element.index = r.readU16();
                decl.push_back(element);
            }
        }
        else if (header.id == M_GEOMETRY_VERTEX_BUFFER)
        {
            if (childEnd - r.pos < 4) return r.fail("vertex buffer is truncated");
            VertexBufferBind bind;
            bind.source = r.readU16();
            bind.vertexSize = r.readU16();
            ChunkHeader dataHeader;
            std::size_t dataEnd = 0;
            if (!readChunkHeader(r, childEnd, dataHeader, dataEnd)) return false;
            if (dataHeader.id != M_GEOMETRY_VERTEX_BUFFER_DATA)
                return r.fail("vertex buffer has no data chunk");
            if (bind.vertexSize == 0) return r.fail("vertex size is zero");
            const std::size_t dataLength = dataEnd - r.pos;
            if (dataLength != static_cast<std::size_t>(vertexCount) * bind.vertexSize)
                return r.fail("vertex buffer size does not match the vertex count");
            bind.data.resize(dataLength);
            if (!r.take(bind.data.data(), dataLength)) return false;
            if (r.pos != childEnd) return r.fail("vertex buffer has trailing data");
            bool replaced = false;
            for (std::size_t i = 0; i < buffers.size(); ++i)
            {
                if (buffers[i].source == bind.source)
                {
                    buffers[i] = static_cast<VertexBufferBind&&>(bind);
                    replaced = true;
                    break;
                }
            }
            if (!replaced) buffers.push_back(static_cast<VertexBufferBind&&>(bind));
        }
        else
            r.seek(childEnd);
    }
    if (r.error) return false;

    const VertexElementDecl* positionElement = nullptr;
    const VertexElementDecl* normalElement = nullptr;
    const VertexElementDecl* tangentElement = nullptr;
    const VertexElementDecl* colorElement = nullptr;
    const VertexElementDecl* uvElement = nullptr;
    const VertexElementDecl* uv2Element = nullptr;
    for (std::size_t i = 0; i < decl.size(); ++i)
    {
        const VertexElementDecl& e = decl[i];
        if (e.semantic == VES_POSITION) positionElement = &e;
        else if (e.semantic == VES_NORMAL)
            normalElement = &e;
        else if (e.semantic == VES_TANGENT)
            tangentElement = &e;
        else if (e.semantic == VES_DIFFUSE)
            colorElement = &e;
        else if (e.semantic == VES_TEXTURE_COORDINATES)
        {
            if (e.index == 0) uvElement = &e;
            else if (e.index == 1)
                uv2Element = &e;
        }
    }
    if (!positionElement) return r.fail("geometry has no position element");

    for (std::size_t i = 0; i < decl.size(); ++i)
    {
        const VertexElementDecl& e = decl[i];
        const std::size_t elementSize = vertexElementSize(e.type);
        const VertexBufferBind* buffer = findBuffer(buffers, &e);
        if (elementSize != 0 &&
                (!buffer || e.offset > buffer->vertexSize ||
                        elementSize > static_cast<std::size_t>(buffer->vertexSize) - e.offset))
            return r.fail("vertex element exceeds its vertex buffer stride");
    }
    if (positionElement->type != VET_FLOAT3) return r.fail("position element is not float3");
    if (normalElement && normalElement->type != VET_FLOAT3)
        return r.fail("normal element is not float3");

    if (tangentElement && tangentElement->type != VET_FLOAT3 && tangentElement->type != VET_FLOAT4)
        tangentElement = nullptr;
    if (colorElement && colorElement->type != VET_COLOUR && colorElement->type != VET_COLOUR_ARGB &&
            colorElement->type != VET_COLOUR_ABGR)
        colorElement = nullptr;
    if (uvElement && uvElement->type != VET_FLOAT2) uvElement = nullptr;
    if (uv2Element && uv2Element->type != VET_FLOAT2) uv2Element = nullptr;

    const VertexBufferBind* positionBuffer = findBuffer(buffers, positionElement);
    if (!positionBuffer) return r.fail("position source has no vertex buffer");
    const VertexBufferBind* normalBuffer = findBuffer(buffers, normalElement);
    const VertexBufferBind* tangentBuffer = findBuffer(buffers, tangentElement);
    const VertexBufferBind* colorBuffer = findBuffer(buffers, colorElement);
    const VertexBufferBind* uvBuffer = findBuffer(buffers, uvElement);
    const VertexBufferBind* uv2Buffer = findBuffer(buffers, uv2Element);

    layout = GeometryLayout();
    layout.hasNormal = normalBuffer != nullptr;
    layout.hasTangent = tangentBuffer != nullptr;
    layout.hasColor = colorBuffer != nullptr;
    layout.hasUv = uvBuffer != nullptr;
    layout.hasUv2 = uv2Buffer != nullptr;

    surface = SurfaceData();
    surface.positions.resize(vertexCount);
    if (layout.hasNormal) surface.normals.resize(vertexCount);
    if (layout.hasTangent) surface.tangents.resize(vertexCount);
    if (layout.hasColor) surface.colors.resize(vertexCount);
    if (layout.hasUv) surface.uvs.resize(vertexCount);
    if (layout.hasUv2) surface.uvs2.resize(vertexCount);

    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        memcpy(&surface.positions[i], elementAt(positionBuffer, positionElement, i),
                sizeof(float) * 3);
        if (normalBuffer)
            memcpy(&surface.normals[i], elementAt(normalBuffer, normalElement, i),
                    sizeof(float) * 3);
        if (tangentBuffer)
        {
            float tangent[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
            const std::size_t count = tangentElement->type == VET_FLOAT4 ? 4 : 3;
            memcpy(tangent, elementAt(tangentBuffer, tangentElement, i), sizeof(float) * count);
            surface.tangents[i] = Math::Vec4(tangent[0], tangent[1], tangent[2], tangent[3]);
        }
        if (colorBuffer)
        {
            std::uint32_t packed = 0;
            memcpy(&packed, elementAt(colorBuffer, colorElement, i), sizeof(packed));
            std::uint8_t red;
            std::uint8_t green;
            std::uint8_t blue;
            if (colorElement->type == VET_COLOUR_ARGB)
            {
                red = static_cast<std::uint8_t>((packed >> 16) & 0xFF);
                green = static_cast<std::uint8_t>((packed >> 8) & 0xFF);
                blue = static_cast<std::uint8_t>(packed & 0xFF);
            }
            else
            {
                red = static_cast<std::uint8_t>(packed & 0xFF);
                green = static_cast<std::uint8_t>((packed >> 8) & 0xFF);
                blue = static_cast<std::uint8_t>((packed >> 16) & 0xFF);
            }
            const std::uint8_t alpha = static_cast<std::uint8_t>((packed >> 24) & 0xFF);
            surface.colors[i] =
                    Math::Vec4(static_cast<float>(red) / 255.0f, static_cast<float>(green) / 255.0f,
                            static_cast<float>(blue) / 255.0f, static_cast<float>(alpha) / 255.0f);
        }
        if (uvBuffer) memcpy(&surface.uvs[i], elementAt(uvBuffer, uvElement, i), sizeof(float) * 2);
        if (uv2Buffer)
            memcpy(&surface.uvs2[i], elementAt(uv2Buffer, uv2Element, i), sizeof(float) * 2);
    }
    return true;
}

struct Geometry
{
    SurfaceData surface;
    GeometryLayout layout;
};

struct ParsedSubMesh
{
    ct::String material;
    SurfaceData surface;
    GeometryLayout layout;
    bool shared = false;
    bool skinned = false;
};

struct ParsedMesh
{
    ct::Vector<ParsedSubMesh> submeshes;
    ct::String skeletonLink;
    ct::Vector<BoneAssignment> sharedAssignments;
};

bool triangulate(Cursor& r, std::uint16_t operation, const ct::Vector<std::uint32_t>& raw,
        ct::Vector<std::uint32_t>& out)
{
    out.clear();
    if (operation == OT_TRIANGLE_LIST)
    {
        if (raw.size() % 3 != 0) return r.fail("triangle list index count is not a multiple of 3");
        out = raw;
        return true;
    }
    if (operation == OT_TRIANGLE_STRIP)
    {
        for (std::size_t i = 2; i < raw.size(); ++i)
        {
            const std::uint32_t a = (i & 1) ? raw[i - 1] : raw[i - 2];
            const std::uint32_t b = (i & 1) ? raw[i - 2] : raw[i - 1];
            const std::uint32_t c = raw[i];
            if (a == b || b == c || a == c) continue;
            out.push_back(a);
            out.push_back(b);
            out.push_back(c);
        }
        return true;
    }
    if (operation == OT_TRIANGLE_FAN)
    {
        for (std::size_t i = 2; i < raw.size(); ++i)
        {
            const std::uint32_t a = raw[0];
            const std::uint32_t b = raw[i - 1];
            const std::uint32_t c = raw[i];
            if (a == b || b == c || a == c) continue;
            out.push_back(a);
            out.push_back(b);
            out.push_back(c);
        }
        return true;
    }
    return r.fail("submesh operation is not a triangle list, strip or fan");
}

bool readSubMesh(Cursor& r, std::size_t endPos, std::size_t meshEnd, const Geometry* shared,
        bool legacy, ParsedSubMesh& out)
{
    if (!readOgreString(r, endPos, out.material) || endPos - r.pos < 6)
        return r.fail("submesh is truncated");
    const bool useSharedVertices = r.readBool();
    const std::uint32_t indexCount = r.readU32();
    const bool indexes32Bit = r.readBool();

    if (indexCount > kMaxIndices) return r.fail("submesh index count is invalid");
    const std::size_t indexSize = indexes32Bit ? sizeof(std::uint32_t) : sizeof(std::uint16_t);
    if (static_cast<std::size_t>(indexCount) > (endPos - r.pos) / indexSize)
        return r.fail("submesh indices exceed the chunk");

    ct::Vector<std::uint32_t> rawIndices;
    rawIndices.resize(indexCount);
    for (std::uint32_t i = 0; i < indexCount; ++i)
        rawIndices[i] = indexes32Bit ? r.readU32() : static_cast<std::uint32_t>(r.readU16());

    ct::Vector<BoneAssignment> localAssignments;
    out.shared = useSharedVertices;

    if (useSharedVertices)
    {
        if (!shared) return r.fail("submesh uses shared vertices but the mesh has none");
        out.surface = shared->surface;
        out.layout = shared->layout;
    }
    else
    {
        if (r.pos >= endPos) return r.fail("submesh has no dedicated geometry");
        ChunkHeader geometryHeader;
        std::size_t geometryEnd = 0;
        if (legacy ? !readChunkHeaderLoose(r, endPos, geometryHeader, geometryEnd)
                   : !readChunkHeader(r, endPos, geometryHeader, geometryEnd))
            return false;
        if (geometryHeader.id != M_GEOMETRY) return r.fail("submesh geometry chunk is missing");
        if (!readGeometry(r, geometryEnd, out.surface, out.layout)) return false;
    }

    std::uint16_t operation = OT_TRIANGLE_LIST;
    while (r.pos < meshEnd)
    {
        const std::size_t headerPosition = r.pos;
        ChunkHeader optional;
        std::size_t optionalEnd = 0;
        if (legacy ? !readChunkHeaderLoose(r, meshEnd, optional, optionalEnd)
                   : !readChunkHeader(r, meshEnd, optional, optionalEnd))
            return false;
        if (optional.id != M_SUBMESH_BONE_ASSIGNMENT && optional.id != M_SUBMESH_OPERATION &&
                optional.id != M_SUBMESH_TEXTURE_ALIAS)
        {
            r.seek(headerPosition);
            break;
        }
        if (optional.id == M_SUBMESH_BONE_ASSIGNMENT)
        {
            BoneAssignment assignment;
            if (!readBoneAssignment(r, optionalEnd, assignment)) return false;
            localAssignments.push_back(assignment);
        }
        else if (optional.id == M_SUBMESH_OPERATION)
        {
            if (optionalEnd - r.pos != sizeof(std::uint16_t))
                return r.fail("submesh operation chunk is invalid");
            operation = r.readU16();
        }
        else
        {
            ct::String alias;
            ct::String texture;
            if (!readOgreString(r, optionalEnd, alias) || !readOgreString(r, optionalEnd, texture))
                return false;
        }
        r.seek(optionalEnd);
    }

    if (!triangulate(r, operation, rawIndices, out.surface.indices)) return false;
    const std::size_t vertexCount = out.surface.positions.size();
    for (std::size_t i = 0; i < out.surface.indices.size(); ++i)
    {
        if (out.surface.indices[i] >= vertexCount) return r.fail("submesh index is out of range");
    }

    if (!useSharedVertices)
    {
        out.skinned = !localAssignments.empty();
        if (out.skinned) applyAssignments(out.surface, localAssignments);
    }
    return !r.error;
}

bool skipLegacyLod(Cursor& r, std::size_t meshEnd, bool hasStrategyName, std::size_t submeshCount)
{
    if (hasStrategyName)
    {
        ct::String strategy;
        if (!readOgreString(r, meshEnd, strategy)) return false;
    }
    if (!r.canRead(3)) return r.fail("lod chunk is truncated");
    const std::uint16_t levels = r.readU16();
    const bool manual = r.readBool();
    for (std::uint16_t level = 1; level < levels; ++level)
    {
        ChunkHeader usage;
        std::size_t usageEnd = 0;
        if (!readChunkHeaderLoose(r, meshEnd, usage, usageEnd)) return false;
        if (usage.id != M_MESH_LOD_USAGE || !r.canRead(sizeof(float)))
            return r.fail("lod usage chunk is missing");
        r.readF32();
        if (manual)
        {
            ChunkHeader detail;
            std::size_t detailEnd = 0;
            if (!readChunkHeaderLoose(r, meshEnd, detail, detailEnd)) return false;
            if (detail.id != M_MESH_LOD_MANUAL) return r.fail("lod manual chunk is missing");
            ct::String name;
            if (!readOgreString(r, meshEnd, name)) return false;
            continue;
        }
        for (std::size_t sub = 0; sub < submeshCount; ++sub)
        {
            ChunkHeader generated;
            std::size_t generatedEnd = 0;
            if (!readChunkHeaderLoose(r, meshEnd, generated, generatedEnd)) return false;
            if (generated.id != M_MESH_LOD_GENERATED || !r.canRead(5))
                return r.fail("lod generated chunk is missing");
            const std::uint32_t indexCount = r.readU32();
            const bool indexes32Bit = r.readBool();
            const std::size_t bytes =
                    static_cast<std::size_t>(indexCount) * (indexes32Bit ? 4u : 2u);
            if (!r.canRead(bytes)) return r.fail("lod indices are truncated");
            r.seek(r.pos + bytes);
        }
    }
    return !r.error;
}

bool parseMesh(const std::uint8_t* data, std::size_t size, ParsedMesh& mesh, const char*& error)
{
    Cursor r;
    r.data = data;
    r.size = size;

    const bool parsed = [&]()
    {
        if (!r.canRead(sizeof(std::uint16_t))) return r.fail("data is truncated before its header");
        if (r.readU16() != M_HEADER) return r.fail("not an Ogre mesh file");
        ct::String version;
        if (!readOgreString(r, r.size, version)) return false;
        if (version != "[MeshSerializer_v1.100]" && version != "[MeshSerializer_v1.8]" &&
                version != "[MeshSerializer_v1.41]" && version != "[MeshSerializer_v1.40]")
            return r.fail("unsupported mesh version");

        const bool legacy = version != "[MeshSerializer_v1.100]";
        const bool lodStrategyName = version != "[MeshSerializer_v1.40]";
        ChunkHeader meshHeader;
        std::size_t meshEnd = 0;
        if (legacy ? !readChunkHeaderLoose(r, r.size, meshHeader, meshEnd)
                   : !readChunkHeader(r, r.size, meshHeader, meshEnd))
            return false;
        if (meshHeader.id != M_MESH) return r.fail("mesh chunk is missing");
        if (legacy) meshEnd = r.size;
        if (r.pos >= meshEnd) return r.fail("mesh chunk is empty");
        r.readBool();

        Geometry shared;
        bool hasShared = false;

        while (r.pos < meshEnd)
        {
            ChunkHeader header;
            std::size_t childEnd = 0;
            if (legacy ? !readChunkHeaderLoose(r, meshEnd, header, childEnd)
                       : !readChunkHeader(r, meshEnd, header, childEnd))
                return false;

            if (header.id == M_GEOMETRY)
            {
                if (!readGeometry(r, childEnd, shared.surface, shared.layout)) return false;
                hasShared = true;
            }
            else if (header.id == M_SUBMESH)
            {
                ParsedSubMesh submesh;
                if (!readSubMesh(r, legacy ? meshEnd : childEnd, meshEnd,
                            hasShared ? &shared : nullptr, legacy, submesh))
                    return false;
                mesh.submeshes.push_back(static_cast<ParsedSubMesh&&>(submesh));
            }
            else if (header.id == M_MESH_LOD_LEVEL && legacy)
            {
                if (!skipLegacyLod(r, meshEnd, lodStrategyName, mesh.submeshes.size()))
                    return false;
            }
            else if (header.id == M_MESH_BONE_ASSIGNMENT)
            {
                BoneAssignment assignment;
                if (!readBoneAssignment(r, childEnd, assignment)) return false;
                mesh.sharedAssignments.push_back(assignment);
            }
            else if (header.id == M_MESH_SKELETON_LINK)
            {
                if (!readOgreString(r, childEnd, mesh.skeletonLink) || r.pos != childEnd)
                    return r.fail("skeleton link is invalid");
            }
            else
                r.seek(childEnd);
        }
        if (r.error) return false;

        for (std::size_t i = 0; i < mesh.submeshes.size(); ++i)
        {
            ParsedSubMesh& submesh = mesh.submeshes[i];
            if (!submesh.shared) continue;
            submesh.skinned = !mesh.sharedAssignments.empty();
            if (submesh.skinned) applyAssignments(submesh.surface, mesh.sharedAssignments);
        }
        if (mesh.submeshes.empty()) return r.fail("mesh has no submeshes");
        return true;
    }();

    if (!parsed)
    {
        error = r.error ? r.error : "mesh is invalid";
        return false;
    }
    return true;
}

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') slash = i;
    }
    if (slash == ct::String::npos) return ct::String();
    return ct::String(path.c_str(), slash + 1);
}

ct::String baseNameOf(const ct::String& path)
{
    std::size_t begin = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') begin = i + 1;
    }
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

bool loadSkeletonBytes(const ct::String& meshPath, const ct::String& link,
        ct::Vector<std::uint8_t>& bytes)
{
    ct::String normalized = link;
    for (std::size_t i = 0; i < normalized.size(); ++i)
    {
        if (normalized[i] == '\\') normalized[i] = '/';
    }
    if (!normalized.ends_with(".skeleton")) normalized += ".skeleton";

    const ct::String directory = directoryOf(meshPath);
    const ct::String relative = directory + normalized;
    std::size_t linkSlash = ct::String::npos;
    for (std::size_t i = 0; i < normalized.size(); ++i)
    {
        if (normalized[i] == '/') linkSlash = i;
    }
    const ct::String baseName =
            linkSlash == ct::String::npos ? normalized : normalized.substr(linkSlash + 1);
    const ct::String besideMesh = directory + baseName;

    const FileSystem& files = FileSystem::getSingleton();
    const ct::String candidates[4] = { relative, normalized, besideMesh, baseName };
    ct::String chosen;
    for (std::size_t i = 0; i < 4; ++i)
    {
        if (files.exists(candidates[i]))
        {
            chosen = candidates[i];
            break;
        }
    }
    if (chosen.empty()) chosen = relative;
    bytes = files.readBinary(chosen);
    return !bytes.empty();
}

std::uint64_t surfaceBytes(const SurfaceData& surface)
{
    std::uint64_t total = 0;
    total += surface.positions.size() * sizeof(Math::Vec3);
    total += surface.normals.size() * sizeof(Math::Vec3);
    total += surface.tangents.size() * sizeof(Math::Vec4);
    total += surface.colors.size() * sizeof(Math::Vec4);
    total += surface.uvs.size() * sizeof(Math::Vec2);
    total += surface.uvs2.size() * sizeof(Math::Vec2);
    total += surface.joints.size() * sizeof(std::uint32_t);
    total += surface.weights.size() * sizeof(Math::Vec4);
    total += surface.indices.size() * sizeof(std::uint32_t);
    return total;
}

bool mapJoints(SurfaceData& surface, const SkeletonResult& skeleton, const char*& error)
{
    for (std::size_t v = 0; v < surface.joints.size(); ++v)
    {
        const float weights[4] = { surface.weights[v].x, surface.weights[v].y, surface.weights[v].z,
            surface.weights[v].w };
        std::uint32_t packed = 0;
        for (std::size_t c = 0; c < 4; ++c)
        {
            if (weights[c] <= 0.0f) continue;
            const std::uint32_t handle = (surface.joints[v] >> (8 * c)) & 0xFFu;
            const int bone = skeleton.handleToBone[handle];
            if (bone < 0)
            {
                error = "mesh references a missing bone handle";
                return false;
            }
            packed |= static_cast<std::uint32_t>(bone) << (8 * c);
        }
        surface.joints[v] = packed;
    }
    return true;
}

struct OgreSource : ModelSource
{
    ct::Vector<SurfaceData> surfaces;
};

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    const char* error = "";
    ParsedMesh parsed;
    if (!parseMesh(data, size, parsed, error)) return fail(out, error);

    const bool linked = !parsed.skeletonLink.empty();
    SkeletonResult skeleton;
    if (linked)
    {
        ct::Vector<std::uint8_t> skeletonBytes;
        if (!loadSkeletonBytes(path, parsed.skeletonLink, skeletonBytes))
            return fail(out, "linked skeleton file is missing");
        if (!loadSkeleton(skeletonBytes, skeleton, error)) return fail(out, error);
    }

    bool anySkinned = false;
    for (std::size_t i = 0; i < parsed.submeshes.size(); ++i)
    {
        ParsedSubMesh& submesh = parsed.submeshes[i];
        if (linked && submesh.skinned)
        {
            if (!mapJoints(submesh.surface, skeleton, error)) return fail(out, error);
            anySkinned = true;
        }
        else
        {
            submesh.skinned = false;
            submesh.surface.joints.clear();
            submesh.surface.weights.clear();
        }
    }

    ct::Unique<OgreSource> owned = ct::make_unique<OgreSource>();
    const ct::String baseName = baseNameOf(path);

    ModelMeshData mesh;
    mesh.name = baseName;
    for (std::size_t i = 0; i < parsed.submeshes.size(); ++i)
    {
        ParsedSubMesh& submesh = parsed.submeshes[i];

        int material = -1;
        for (std::size_t m = 0; m < out.materials.size(); ++m)
        {
            if (out.materials[m].name == submesh.material)
            {
                material = static_cast<int>(m);
                break;
            }
        }
        if (material < 0)
        {
            ModelMaterialData data;
            data.name = submesh.material;
            out.materials.push_back(data);
            material = static_cast<int>(out.materials.size() - 1);
        }

        ModelPrimitiveData primitive;
        primitive.material = material;
        primitive.attributeMask = kVertexPosition;
        if (submesh.layout.hasNormal) primitive.attributeMask |= kVertexNormal;
        if (submesh.layout.hasTangent) primitive.attributeMask |= kVertexTangent;
        if (submesh.layout.hasColor) primitive.attributeMask |= kVertexColor;
        if (submesh.layout.hasUv) primitive.attributeMask |= kVertexUv;
        if (submesh.layout.hasUv2) primitive.attributeMask |= kVertexUv2;
        if (submesh.skinned) primitive.attributeMask |= kVertexJoints | kVertexWeights;
        primitive.vertexCount = static_cast<std::uint32_t>(submesh.surface.positions.size());
        primitive.indexCount = static_cast<std::uint32_t>(submesh.surface.indices.size());
        primitive.sourceBytes = surfaceBytes(submesh.surface);
        primitive.source = static_cast<std::uint32_t>(owned->surfaces.size());
        primitive.aabb = MeshStore::computeAabb(submesh.surface.positions);
        primitive.hasAabb = true;
        primitive.skinned = submesh.skinned;

        out.geometryBytes += primitive.sourceBytes;
        ++out.primitiveCount;
        mesh.primitives.push_back(primitive);
        submesh.surface.topology = prisma::Topology::Triangles;
        owned->surfaces.push_back(static_cast<SurfaceData&&>(submesh.surface));
    }
    out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));

    ModelNodeData node;
    node.name = baseName;
    node.mesh = 0;
    node.skeleton = anySkinned ? 0 : -1;
    out.nodes.push_back(static_cast<ModelNodeData&&>(node));
    out.roots.push_back(0);

    if (linked)
    {
        out.skeletons.push_back(static_cast<Skeleton&&>(skeleton.skeleton));
        out.animations = static_cast<ct::Vector<AnimationClip>&&>(skeleton.clips);
    }

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<OgreSource>&&>(owned));
    return true;
}

} // namespace

const char* OgreMeshImporter::name() const
{
    return "Ogre mesh";
}

bool OgreMeshImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "mesh") == 0;
}

bool OgreMeshImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(bytes.data(), bytes.size(), path, out);
}

bool OgreMeshImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty mesh data");
    return build(data, size, virtualName, out);
}

bool OgreMeshImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const OgreSource* source = static_cast<const OgreSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool OgreMeshImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
