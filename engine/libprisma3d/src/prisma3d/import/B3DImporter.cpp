#include "prisma3d/import/B3DImporter.h"

#include "prisma3d/core/FileSystem.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::size_t kMaxStringLength = 4096;
const std::size_t kMaxNodes = 65535;
const std::size_t kMaxVertices = 16u * 1024u * 1024u;
const std::int32_t kMaxTextureSets = 8;
const std::int32_t kMaxTextureSetSize = 4;
const std::int32_t kMaxBrushTextures = 8;
const int kMaxDepth = 512;
const int kMaxJoint = 255;
const float kFallbackFps = 25.0f;
const float kKeyTolerance = 1e-4f;

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

    bool ensure(std::size_t end, std::size_t count)
    {
        if (error) return false;
        if (end > size || pos > end || end - pos < count) return fail("data is truncated");
        return true;
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

    std::uint32_t readU32()
    {
        std::uint8_t raw[4] = { 0, 0, 0, 0 };
        take(raw, 4);
        return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8) |
               (static_cast<std::uint32_t>(raw[2]) << 16) |
               (static_cast<std::uint32_t>(raw[3]) << 24);
    }

    std::int32_t readS32()
    {
        return static_cast<std::int32_t>(readU32());
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

struct Chunk
{
    char tag[4] = { 0, 0, 0, 0 };
    std::size_t end = 0;

    bool is(const char* name) const
    {
        return memcmp(tag, name, 4) == 0;
    }
};

bool readChunk(Cursor& r, std::size_t parentEnd, Chunk& chunk)
{
    if (r.error) return false;
    if (parentEnd > r.size || r.pos > parentEnd || parentEnd - r.pos < 8)
        return r.fail("chunk header is truncated");
    r.take(chunk.tag, 4);
    const std::int32_t length = r.readS32();
    if (length < 0) return r.fail("chunk length is invalid");
    if (static_cast<std::size_t>(length) > parentEnd - r.pos)
        return r.fail("chunk exceeds its parent");
    chunk.end = r.pos + static_cast<std::size_t>(length);
    return true;
}

bool readCString(Cursor& r, std::size_t end, ct::String& value)
{
    if (r.error) return false;
    if (end > r.size || r.pos > end) return r.fail("string is out of range");
    const std::size_t available = end - r.pos;
    const std::size_t scanLength =
            available < kMaxStringLength + 1 ? available : kMaxStringLength + 1;
    const std::uint8_t* begin = r.data + r.pos;
    const void* terminator = memchr(begin, 0, scanLength);
    if (!terminator) return r.fail("string is not terminated");
    const std::size_t length =
            static_cast<std::size_t>(static_cast<const std::uint8_t*>(terminator) - begin);
    value = ct::String(reinterpret_cast<const char*>(begin), length);
    r.pos += length + 1;
    return true;
}

bool readVec3(Cursor& r, std::size_t end, Math::Vec3& value)
{
    if (!r.ensure(end, sizeof(float) * 3)) return false;
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    if (!finite(value.x) || !finite(value.y) || !finite(value.z))
        return r.fail("vector is not finite");
    return true;
}

bool readQuat(Cursor& r, std::size_t end, Math::Quaternion& value)
{
    if (!r.ensure(end, sizeof(float) * 4)) return false;
    const float w = r.readF32();
    const float x = r.readF32();
    const float y = r.readF32();
    const float z = r.readF32();
    if (!finite(w) || !finite(x) || !finite(y) || !finite(z))
        return r.fail("quaternion is not finite");
    const Math::Quaternion raw(x, y, z, -w);
    const float length = raw.Length();
    value = length > 1e-8f ? raw * (1.0f / length) : Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    return true;
}

struct WeightSlots
{
    int ids[4] = { -1, -1, -1, -1 };
    float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    void add(int bone, float weight)
    {
        if (bone < 0 || weight <= 0.0f) return;
        if (weight > weights[0])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = ids[1];
            weights[2] = weights[1];
            ids[1] = ids[0];
            weights[1] = weights[0];
            ids[0] = bone;
            weights[0] = weight;
        }
        else if (weight > weights[1])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = ids[1];
            weights[2] = weights[1];
            ids[1] = bone;
            weights[1] = weight;
        }
        else if (weight > weights[2])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = bone;
            weights[2] = weight;
        }
        else if (weight > weights[3])
        {
            ids[3] = bone;
            weights[3] = weight;
        }
    }
};

struct Brush
{
    ct::String name;
    Math::Vec3 diffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
    int texture = -1;
    int lightmap = -1;
};

const std::int32_t kTextureSecondUvSet = 1 << 16;

struct TriGroup
{
    int brush = -1;
    ct::Vector<std::uint32_t> indices;
};

struct ParsedMesh
{
    bool hasNormal = false;
    bool hasColor = false;
    bool hasUv = false;
    bool hasUv2 = false;
    bool haveVertices = false;
    bool skinned = false;
    std::size_t vertexStart = 0;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<Math::Vec4> colors;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<Math::Vec2> uvs2;
    ct::Vector<TriGroup> groups;
    ct::Vector<WeightSlots> slots;
};

struct VecKey
{
    float frame = 0.0f;
    Math::Vec3 value = Math::Vec3(0.0f, 0.0f, 0.0f);
};

struct QuatKey
{
    float frame = 0.0f;
    Math::Quaternion value = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
};

struct ParsedNode
{
    ct::String name;
    int parent = -1;
    int mesh = -1;
    Math::Vec3 translation = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion rotation = Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Mat4 global = Math::Mat4::Identity();
    ct::Vector<VecKey> positionKeys;
    ct::Vector<VecKey> scaleKeys;
    ct::Vector<QuatKey> rotationKeys;
};

struct Assignment
{
    std::size_t vertex = 0;
    int bone = 0;
    float weight = 0.0f;
};

struct Context
{
    Cursor r;
    ct::Vector<ct::String> textures;
    ct::Vector<std::int32_t> textureFlags;
    ct::Vector<Brush> brushes;
    ct::Vector<ParsedNode> nodes;
    ct::Vector<ParsedMesh> meshes;
    ct::Vector<Assignment> assignments;
    std::size_t totalVertices = 0;
    bool sawBones = false;
    bool sawAnim = false;
    std::int32_t animFrames = 0;
    float animFps = kFallbackFps;
};

bool readVertices(Context& c, std::size_t end, ParsedMesh& mesh)
{
    Cursor& r = c.r;
    if (mesh.haveVertices) return r.fail("mesh has more than one vertex chunk");
    if (!r.ensure(end, 12)) return false;
    const std::int32_t flags = r.readS32();
    const std::int32_t sets = r.readS32();
    const std::int32_t setSize = r.readS32();
    if (sets < 0 || sets > kMaxTextureSets || setSize < 0 || setSize > kMaxTextureSetSize)
        return r.fail("vertex layout is invalid");
    const bool hasNormal = (flags & 1) != 0;
    const bool hasColor = (flags & 2) != 0;
    std::size_t stride = 12;
    if (hasNormal) stride += 12;
    if (hasColor) stride += 16;
    stride += static_cast<std::size_t>(sets) * static_cast<std::size_t>(setSize) * 4;

    const std::size_t remaining = end - r.pos;
    if (remaining % stride != 0) return r.fail("vertex data has an invalid size");
    const std::size_t count = remaining / stride;
    if (count > kMaxVertices || c.totalVertices + count > kMaxVertices)
        return r.fail("mesh has too many vertices");

    mesh.haveVertices = true;
    mesh.hasNormal = hasNormal;
    mesh.hasColor = hasColor;
    mesh.hasUv = sets >= 1;
    mesh.hasUv2 = sets >= 2;
    mesh.positions.reserve(count);
    if (hasNormal) mesh.normals.reserve(count);
    if (hasColor) mesh.colors.reserve(count);
    if (mesh.hasUv) mesh.uvs.reserve(count);
    if (mesh.hasUv2) mesh.uvs2.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        Math::Vec3 position;
        if (!readVec3(r, end, position)) return false;
        mesh.positions.push_back(position);
        if (hasNormal)
        {
            Math::Vec3 normal;
            if (!readVec3(r, end, normal)) return false;
            mesh.normals.push_back(normal);
        }
        if (hasColor)
        {
            Math::Vec4 color;
            color.x = r.readF32();
            color.y = r.readF32();
            color.z = r.readF32();
            color.w = r.readF32();
            mesh.colors.push_back(color);
        }
        for (std::int32_t set = 0; set < sets; ++set)
        {
            float u = 0.0f;
            float v = 0.0f;
            for (std::int32_t k = 0; k < setSize; ++k)
            {
                const float value = r.readF32();
                if (k == 0) u = value;
                if (k == 1) v = value;
            }
            if (set == 0) mesh.uvs.push_back(Math::Vec2(u, v));
            if (set == 1) mesh.uvs2.push_back(Math::Vec2(u, v));
        }
    }
    return !r.error;
}

bool readTriangles(Context& c, std::size_t end, ParsedMesh& mesh)
{
    Cursor& r = c.r;
    if (!r.ensure(end, 4)) return false;
    TriGroup group;
    group.brush = r.readS32();
    const std::size_t remaining = end - r.pos;
    if (remaining % 12 != 0) return r.fail("triangle data has an invalid size");
    const std::size_t count = remaining / 3 / 4;
    group.indices.reserve(count * 3);
    for (std::size_t i = 0; i < count * 3; ++i)
    {
        const std::int32_t index = r.readS32();
        if (index < 0) return r.fail("triangle index is invalid");
        group.indices.push_back(static_cast<std::uint32_t>(index));
    }
    if (r.error) return false;
    if (!group.indices.empty()) mesh.groups.push_back(static_cast<TriGroup&&>(group));
    return true;
}

bool readMesh(Context& c, std::size_t end, ParsedMesh& mesh)
{
    Cursor& r = c.r;
    if (!r.ensure(end, 4)) return false;
    r.readS32();
    while (r.pos < end)
    {
        Chunk chunk;
        if (!readChunk(r, end, chunk)) return false;
        if (chunk.is("VRTS"))
        {
            if (!readVertices(c, chunk.end, mesh)) return false;
        }
        else if (chunk.is("TRIS"))
        {
            if (!readTriangles(c, chunk.end, mesh)) return false;
        }
        r.seek(chunk.end);
    }
    for (std::size_t g = 0; g < mesh.groups.size(); ++g)
    {
        const ct::Vector<std::uint32_t>& indices = mesh.groups[g].indices;
        for (std::size_t i = 0; i < indices.size(); ++i)
        {
            if (indices[i] >= mesh.positions.size())
                return r.fail("triangle index is out of range");
        }
    }
    return !r.error;
}

bool readKeys(Context& c, std::size_t end, std::size_t nodeIndex)
{
    Cursor& r = c.r;
    if (!r.ensure(end, 4)) return false;
    const std::int32_t flags = r.readS32();
    const bool hasPosition = (flags & 1) != 0;
    const bool hasScale = (flags & 2) != 0;
    const bool hasRotation = (flags & 4) != 0;
    std::size_t keySize = 4;
    if (hasPosition) keySize += 12;
    if (hasScale) keySize += 12;
    if (hasRotation) keySize += 16;
    const std::size_t remaining = end - r.pos;
    if (remaining % keySize != 0) return r.fail("key data has an invalid size");
    const std::size_t count = remaining / keySize;

    for (std::size_t i = 0; i < count; ++i)
    {
        const float frame = static_cast<float>(r.readS32());
        if (hasPosition)
        {
            VecKey key;
            key.frame = frame;
            if (!readVec3(r, end, key.value)) return false;
            ct::Vector<VecKey>& keys = c.nodes[nodeIndex].positionKeys;
            if (!keys.empty() && frame < keys.back().frame)
                return r.fail("keys are not in time order");
            keys.push_back(key);
        }
        if (hasScale)
        {
            VecKey key;
            key.frame = frame;
            if (!readVec3(r, end, key.value)) return false;
            ct::Vector<VecKey>& keys = c.nodes[nodeIndex].scaleKeys;
            if (!keys.empty() && frame < keys.back().frame)
                return r.fail("keys are not in time order");
            keys.push_back(key);
        }
        if (hasRotation)
        {
            QuatKey key;
            key.frame = frame;
            if (!readQuat(r, end, key.value)) return false;
            ct::Vector<QuatKey>& keys = c.nodes[nodeIndex].rotationKeys;
            if (!keys.empty() && frame < keys.back().frame)
                return r.fail("keys are not in time order");
            keys.push_back(key);
        }
    }
    return !r.error;
}

LocalPose localPose(const ParsedNode& node)
{
    LocalPose pose;
    pose.position = node.translation;
    pose.rotation = node.rotation;
    pose.scale = node.scale;
    return pose;
}

bool parseNode(Context& c, std::size_t end, int parent, const Math::Mat4& parentGlobal, int depth)
{
    Cursor& r = c.r;
    if (depth > kMaxDepth) return r.fail("nodes are nested too deeply");
    if (c.nodes.size() >= kMaxNodes) return r.fail("file has too many nodes");

    ParsedNode node;
    node.parent = parent;
    if (!readCString(r, end, node.name)) return false;
    if (!readVec3(r, end, node.translation) || !readVec3(r, end, node.scale) ||
            !readQuat(r, end, node.rotation))
        return false;
    node.global = parentGlobal * localPose(node).toMatrix();
    const Math::Mat4 global = node.global;

    const std::size_t index = c.nodes.size();
    c.nodes.push_back(static_cast<ParsedNode&&>(node));
    std::size_t nodeVertexStart = 0;

    while (r.pos < end)
    {
        Chunk chunk;
        if (!readChunk(r, end, chunk)) return false;
        if (chunk.is("MESH"))
        {
            std::size_t owner = index;
            if (c.nodes[index].mesh >= 0)
            {
                if (c.nodes.size() >= kMaxNodes) return r.fail("file has too many nodes");
                char suffix[16];
                snprintf(suffix, sizeof(suffix), "_%u",
                        static_cast<unsigned>(c.nodes.size() - index));
                ParsedNode extra;
                extra.name = c.nodes[index].name + suffix;
                extra.parent = static_cast<int>(index);
                extra.global = global;
                owner = c.nodes.size();
                c.nodes.push_back(static_cast<ParsedNode&&>(extra));
            }
            ParsedMesh mesh;
            mesh.vertexStart = c.totalVertices;
            if (!readMesh(c, chunk.end, mesh)) return false;
            nodeVertexStart = mesh.vertexStart;
            c.totalVertices += mesh.positions.size();
            c.nodes[owner].mesh = static_cast<int>(c.meshes.size());
            c.meshes.push_back(static_cast<ParsedMesh&&>(mesh));
        }
        else if (chunk.is("BONE"))
        {
            c.sawBones = true;
            if ((chunk.end - r.pos) % 8 != 0) return r.fail("bone data has an invalid size");
            while (r.pos < chunk.end)
            {
                const std::int32_t vertex = r.readS32();
                const float weight = r.readF32();
                if (r.error) return false;
                if (vertex < 0 || !finite(weight) || weight <= 0.0f) continue;
                Assignment assignment;
                assignment.vertex = nodeVertexStart + static_cast<std::size_t>(vertex);
                assignment.bone = static_cast<int>(index);
                assignment.weight = weight;
                c.assignments.push_back(assignment);
            }
        }
        else if (chunk.is("KEYS"))
        {
            if (!readKeys(c, chunk.end, index)) return false;
        }
        else if (chunk.is("ANIM"))
        {
            if (!r.ensure(chunk.end, 12)) return false;
            r.readS32();
            const std::int32_t frames = r.readS32();
            float fps = r.readF32();
            if (!finite(fps)) return r.fail("animation rate is invalid");
            if (fps <= 0.0f) fps = kFallbackFps;
            c.animFrames = frames;
            c.animFps = fps;
            c.sawAnim = true;
        }
        else if (chunk.is("NODE"))
        {
            if (!parseNode(c, chunk.end, static_cast<int>(index), global, depth + 1)) return false;
        }
        r.seek(chunk.end);
    }
    return !r.error;
}

bool parseTextures(Context& c, std::size_t end)
{
    Cursor& r = c.r;
    while (r.pos < end)
    {
        ct::String name;
        if (!readCString(r, end, name)) return false;
        if (!r.ensure(end, 28)) return false;
        const std::int32_t flags = r.readS32();
        r.seek(r.pos + 24);
        c.textures.push_back(name);
        c.textureFlags.push_back(flags);
    }
    return !r.error;
}

bool parseBrushes(Context& c, std::size_t end)
{
    Cursor& r = c.r;
    if (!r.ensure(end, 4)) return false;
    const std::int32_t textureCount = r.readS32();
    if (textureCount < 0 || textureCount > kMaxBrushTextures)
        return r.fail("brush texture count is invalid");
    while (r.pos < end)
    {
        Brush brush;
        if (!readCString(r, end, brush.name)) return false;
        if (!r.ensure(end, 28 + static_cast<std::size_t>(textureCount) * 4)) return false;
        brush.diffuse.x = r.readF32();
        brush.diffuse.y = r.readF32();
        brush.diffuse.z = r.readF32();
        if (!finite(brush.diffuse.x) || !finite(brush.diffuse.y) || !finite(brush.diffuse.z))
            return r.fail("brush colour is not finite");
        r.seek(r.pos + 16);
        for (std::int32_t i = 0; i < textureCount; ++i)
        {
            const std::int32_t texture = r.readS32();
            if (i == 0) brush.texture = texture;
            else if (brush.lightmap < 0 && texture >= 0 &&
                     static_cast<std::size_t>(texture) < c.textureFlags.size() &&
                     (c.textureFlags[static_cast<std::size_t>(texture)] & kTextureSecondUvSet))
                brush.lightmap = texture;
        }
        c.brushes.push_back(static_cast<Brush&&>(brush));
    }
    return !r.error;
}

bool parseFile(Context& c, const std::uint8_t* data, std::size_t size)
{
    Cursor& r = c.r;
    r.data = data;
    r.size = size;
    if (size < 12) return r.fail("data is truncated");
    if (memcmp(data, "BB3D", 4) != 0) return r.fail("not a Blitz3D file");
    r.pos = 4;
    const std::int32_t length = r.readS32();
    if (length < 4) return r.fail("file length is invalid");
    if (static_cast<std::size_t>(length) > size - r.pos) return r.fail("data is truncated");
    const std::size_t end = r.pos + static_cast<std::size_t>(length);
    r.readS32();

    while (r.pos < end)
    {
        Chunk chunk;
        if (!readChunk(r, end, chunk)) return false;
        if (chunk.is("TEXS"))
        {
            if (!parseTextures(c, chunk.end)) return false;
        }
        else if (chunk.is("BRUS"))
        {
            if (!parseBrushes(c, chunk.end)) return false;
        }
        else if (chunk.is("NODE"))
        {
            if (!parseNode(c, chunk.end, -1, Math::Mat4::Identity(), 0)) return false;
        }
        r.seek(chunk.end);
    }
    return !r.error;
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

ct::String joinPath(const ct::String& directory, const ct::String& name)
{
    if (name.empty() || directory.empty()) return name;
    if (name[0] == '/' || name[0] == '\\' || (name.size() > 1 && name[1] == ':')) return name;
    return directory + name;
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

bool applyAssignments(Context& c)
{
    for (std::size_t m = 0; m < c.meshes.size(); ++m)
        c.meshes[m].slots.resize(c.meshes[m].positions.size());

    for (std::size_t i = 0; i < c.assignments.size(); ++i)
    {
        const Assignment& assignment = c.assignments[i];
        std::size_t low = 0;
        std::size_t high = c.meshes.size();
        while (low < high)
        {
            const std::size_t middle = low + (high - low) / 2;
            if (c.meshes[middle].vertexStart <= assignment.vertex) low = middle + 1;
            else
                high = middle;
        }
        if (low == 0) continue;
        ParsedMesh& mesh = c.meshes[low - 1];
        const std::size_t local = assignment.vertex - mesh.vertexStart;
        if (local >= mesh.positions.size()) continue;
        if (assignment.bone > kMaxJoint)
        {
            c.r.fail("bone index exceeds 255");
            return false;
        }
        mesh.slots[local].add(assignment.bone, assignment.weight);
        mesh.skinned = true;
    }
    return true;
}

void packSlots(const WeightSlots& slots, std::uint32_t& joints, Math::Vec4& weights)
{
    const float sum = slots.weights[0] + slots.weights[1] + slots.weights[2] + slots.weights[3];
    if (sum <= 1e-8f)
    {
        joints = 0;
        weights = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    const int b0 = slots.ids[0] >= 0 ? slots.ids[0] : 0;
    const int b1 = slots.ids[1] >= 0 ? slots.ids[1] : b0;
    const int b2 = slots.ids[2] >= 0 ? slots.ids[2] : b0;
    const int b3 = slots.ids[3] >= 0 ? slots.ids[3] : b0;
    joints = static_cast<std::uint32_t>(b0) | (static_cast<std::uint32_t>(b1) << 8) |
             (static_cast<std::uint32_t>(b2) << 16) | (static_cast<std::uint32_t>(b3) << 24);
    weights = Math::Vec4(slots.weights[0] / sum, slots.weights[1] / sum, slots.weights[2] / sum,
            slots.weights[3] / sum);
}

struct B3DSource : ModelSource
{
    ct::Vector<SurfaceData> surfaces;
};

struct MaterialMap
{
    ct::Vector<int> brushMaterial;
    ct::Vector<int> textureIndex;
    ct::Vector<int> lightmapImage;
    int defaultMaterial = -1;
};

int lightmapImageFor(const Context& c, ModelData& out, MaterialMap& map, int textureId)
{
    if (textureId < 0 || static_cast<std::size_t>(textureId) >= c.textures.size()) return -1;
    const ct::String& file = c.textures[static_cast<std::size_t>(textureId)];
    if (file.empty()) return -1;
    int& slot = map.lightmapImage[static_cast<std::size_t>(textureId)];
    if (slot >= 0) return slot;
    ModelImageData image;
    image.name = file;
    image.path = joinPath(out.directory, file);
    for (std::size_t i = 0; i < out.images.size(); ++i)
    {
        if (out.images[i].path != image.path) continue;
        slot = static_cast<int>(i);
        return slot;
    }
    out.images.push_back(static_cast<ModelImageData&&>(image));
    slot = static_cast<int>(out.images.size() - 1);
    return slot;
}

int textureFor(const Context& c, ModelData& out, MaterialMap& map, int textureId)
{
    if (textureId < 0 || static_cast<std::size_t>(textureId) >= c.textures.size()) return -1;
    const ct::String& file = c.textures[static_cast<std::size_t>(textureId)];
    if (file.empty()) return -1;
    int& slot = map.textureIndex[static_cast<std::size_t>(textureId)];
    if (slot >= 0) return slot;
    ModelImageData image;
    image.name = file;
    image.path = joinPath(out.directory, file);
    out.images.push_back(static_cast<ModelImageData&&>(image));
    ModelTextureData texture;
    texture.image = static_cast<int>(out.images.size() - 1);
    out.textures.push_back(texture);
    slot = static_cast<int>(out.textures.size() - 1);
    return slot;
}

int materialFor(const Context& c, ModelData& out, MaterialMap& map, int brushId)
{
    if (brushId < 0 || static_cast<std::size_t>(brushId) >= c.brushes.size())
    {
        if (map.defaultMaterial < 0)
        {
            ModelMaterialData material;
            material.name = "default";
            material.metallic = 0.0f;
            out.materials.push_back(static_cast<ModelMaterialData&&>(material));
            map.defaultMaterial = static_cast<int>(out.materials.size() - 1);
        }
        return map.defaultMaterial;
    }
    int& slot = map.brushMaterial[static_cast<std::size_t>(brushId)];
    if (slot >= 0) return slot;
    const Brush& brush = c.brushes[static_cast<std::size_t>(brushId)];
    ModelMaterialData material;
    material.name = brush.name;
    material.baseColor = Math::Vec4(brush.diffuse.x, brush.diffuse.y, brush.diffuse.z, 1.0f);
    material.metallic = 0.0f;
    material.baseColorTexture = textureFor(c, out, map, brush.texture);
    out.materials.push_back(static_cast<ModelMaterialData&&>(material));
    slot = static_cast<int>(out.materials.size() - 1);
    return slot;
}

void buildSurface(const ParsedMesh& mesh, const TriGroup& group, SurfaceData& surface)
{
    surface.topology = prisma::Topology::Triangles;
    ct::Vector<std::uint32_t> remap;
    remap.resize(mesh.positions.size(), 0xFFFFFFFFu);
    for (std::size_t i = 0; i < group.indices.size(); ++i)
    {
        const std::uint32_t source = group.indices[i];
        if (remap[source] == 0xFFFFFFFFu)
        {
            remap[source] = static_cast<std::uint32_t>(surface.positions.size());
            surface.positions.push_back(mesh.positions[source]);
            if (mesh.hasNormal) surface.normals.push_back(mesh.normals[source]);
            if (mesh.hasColor) surface.colors.push_back(mesh.colors[source]);
            if (mesh.hasUv) surface.uvs.push_back(mesh.uvs[source]);
            if (mesh.hasUv2) surface.uvs2.push_back(mesh.uvs2[source]);
            if (mesh.skinned)
            {
                std::uint32_t joints = 0;
                Math::Vec4 weights;
                packSlots(mesh.slots[source], joints, weights);
                surface.joints.push_back(joints);
                surface.weights.push_back(weights);
            }
        }
        surface.indices.push_back(remap[source]);
    }
}

ct::String boneName(const Skeleton& skeleton, const ct::String& name, std::size_t index)
{
    if (!name.empty() && skeleton.findBone(name.c_str()) < 0) return name;
    ct::String candidate = name.empty() ? ct::String("node") : name;
    candidate += "_";
    candidate.append_number(index);
    while (skeleton.findBone(candidate.c_str()) >= 0) candidate += "_";
    return candidate;
}

bool buildSkeleton(const Context& c, Skeleton& skeleton)
{
    for (std::size_t i = 0; i < c.nodes.size(); ++i)
    {
        const ParsedNode& node = c.nodes[i];
        const Math::Mat4 local = localPose(node).toMatrix();
        const float determinant = node.global.Determinant();
        Math::Mat4 inverse = Math::Mat4::Identity();
        if (finite(determinant) && fabsf(determinant) > 1e-20f) inverse = node.global.Inverse();
        if (!skeleton.addBone(boneName(skeleton, node.name, i), node.parent, local, inverse))
            return false;
    }
    return skeleton.finalize();
}

Math::Vec3 sampleVec(const ct::Vector<VecKey>& keys, float frame, const Math::Vec3& fallback)
{
    if (keys.empty()) return fallback;
    if (keys.size() == 1 || frame <= keys.front().frame) return keys.front().value;
    if (frame >= keys.back().frame) return keys.back().value;
    std::size_t low = 0;
    std::size_t high = keys.size();
    while (low < high)
    {
        const std::size_t middle = low + (high - low) / 2;
        if (!(frame < keys[middle].frame)) low = middle + 1;
        else
            high = middle;
    }
    const std::size_t next = low;
    const std::size_t previous = next - 1;
    const float span = keys[next].frame - keys[previous].frame;
    const float amount = span > 1e-6f ? (frame - keys[previous].frame) / span : 0.0f;
    return keys[previous].value + (keys[next].value - keys[previous].value) * amount;
}

Math::Quaternion sampleQuat(const ct::Vector<QuatKey>& keys, float frame,
        const Math::Quaternion& fallback)
{
    if (keys.empty()) return fallback;
    if (keys.size() == 1 || frame <= keys.front().frame) return keys.front().value;
    if (frame >= keys.back().frame) return keys.back().value;
    std::size_t low = 0;
    std::size_t high = keys.size();
    while (low < high)
    {
        const std::size_t middle = low + (high - low) / 2;
        if (!(frame < keys[middle].frame)) low = middle + 1;
        else
            high = middle;
    }
    const std::size_t next = low;
    const std::size_t previous = next - 1;
    const float span = keys[next].frame - keys[previous].frame;
    const float amount = span > 1e-6f ? (frame - keys[previous].frame) / span : 0.0f;
    return Math::Quaternion::Slerp(keys[previous].value, keys[next].value, amount);
}

void sortFrames(ct::Vector<float>& frames)
{
    for (std::size_t i = 1; i < frames.size(); ++i)
    {
        const float value = frames[i];
        std::size_t k = i;
        while (k > 0 && frames[k - 1] > value)
        {
            frames[k] = frames[k - 1];
            --k;
        }
        frames[k] = value;
    }
}

bool buildTrack(int boneIndex, const ParsedNode& node, float fps, float firstFrame, float lastFrame,
        BoneTrack& track)
{
    if (node.positionKeys.empty() && node.rotationKeys.empty() && node.scaleKeys.empty())
        return false;

    ct::Vector<float> all;
    all.push_back(firstFrame);
    all.push_back(lastFrame);
    for (std::size_t i = 0; i < node.positionKeys.size(); ++i)
        all.push_back(node.positionKeys[i].frame);
    for (std::size_t i = 0; i < node.scaleKeys.size(); ++i) all.push_back(node.scaleKeys[i].frame);
    for (std::size_t i = 0; i < node.rotationKeys.size(); ++i)
        all.push_back(node.rotationKeys[i].frame);
    sortFrames(all);

    ct::Vector<float> frames;
    for (std::size_t i = 0; i < all.size(); ++i)
    {
        if (all[i] < firstFrame - kKeyTolerance || all[i] > lastFrame + kKeyTolerance) continue;
        if (!frames.empty() && fabsf(all[i] - frames.back()) < kKeyTolerance) continue;
        frames.push_back(all[i]);
    }

    track.bone = boneIndex;
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        const float frame = frames[i];
        track.times.push_back((frame - firstFrame) / fps);
        track.positions.push_back(sampleVec(node.positionKeys, frame, node.translation));
        track.rotations.push_back(sampleQuat(node.rotationKeys, frame, node.rotation));
        track.scales.push_back(sampleVec(node.scaleKeys, frame, node.scale));
    }
    return !track.times.empty();
}

struct Sequence
{
    ct::String name;
    float first = 0.0f;
    float last = 0.0f;
};

bool isSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

void parseSequences(const ct::String& text, ct::Vector<Sequence>& out)
{
    const char* at = text.c_str();
    while (*at)
    {
        const char* end = at;
        while (*end && *end != '\n') ++end;
        const char* cursor = at;
        while (cursor < end && isSpace(*cursor)) ++cursor;
        if (cursor < end && *cursor != '#')
        {
            const char* nameEnd = cursor;
            while (nameEnd < end && !isSpace(*nameEnd)) ++nameEnd;
            Sequence sequence;
            sequence.name = ct::String(cursor, static_cast<std::size_t>(nameEnd - cursor));
            char* next = nullptr;
            sequence.first = strtof(nameEnd, &next);
            const bool haveFirst = next != nameEnd && next <= end;
            const char* lastStart = next;
            sequence.last = strtof(lastStart, &next);
            const bool haveLast = next != lastStart && next <= end;
            if (haveFirst && haveLast && !sequence.name.empty() && sequence.last >= sequence.first)
                out.push_back(static_cast<Sequence&&>(sequence));
        }
        at = *end ? end + 1 : end;
    }
}

ct::String sequencePath(const ct::String& path)
{
    std::size_t dot = path.size();
    for (std::size_t i = path.size(); i > 0; --i)
    {
        const char c = path[i - 1];
        if (c == '/' || c == '\\') break;
        if (c == '.')
        {
            dot = i - 1;
            break;
        }
    }
    return path.substr(0, dot) + ".seq";
}

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path,
        const ct::Vector<Sequence>& sequences, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    Context c;
    if (!parseFile(c, data, size)) return fail(out, c.r.error ? c.r.error : "invalid file");
    if (c.nodes.empty()) return fail(out, "file has no nodes");
    if (!applyAssignments(c)) return fail(out, c.r.error);

    const ct::String baseName = baseNameOf(path);
    ct::Unique<B3DSource> owned = ct::make_unique<B3DSource>();

    MaterialMap map;
    map.brushMaterial.resize(c.brushes.size(), -1);
    map.textureIndex.resize(c.textures.size(), -1);
    map.lightmapImage.resize(c.textures.size(), -1);

    out.nodes.resize(c.nodes.size());
    for (std::size_t i = 0; i < c.nodes.size(); ++i)
    {
        const ParsedNode& source = c.nodes[i];
        ModelNodeData& node = out.nodes[i];
        node.name = source.name;
        node.parent = source.parent;
        node.translation = source.translation;
        node.rotation = source.rotation;
        node.scale = source.scale;
        if (source.parent >= 0)
            out.nodes[static_cast<std::size_t>(source.parent)].children.push_back(
                    static_cast<int>(i));
        else
            out.roots.push_back(static_cast<int>(i));

        if (source.mesh < 0) continue;
        const ParsedMesh& parsed = c.meshes[static_cast<std::size_t>(source.mesh)];
        if (parsed.groups.empty()) continue;

        ModelMeshData mesh;
        mesh.name = source.name.empty() ? baseName : source.name;
        for (std::size_t g = 0; g < parsed.groups.size(); ++g)
        {
            const TriGroup& group = parsed.groups[g];
            SurfaceData surface;
            buildSurface(parsed, group, surface);

            ModelPrimitiveData primitive;
            primitive.material = materialFor(c, out, map, group.brush);
            if (mesh.lightmapImage < 0 && parsed.hasUv2 && group.brush >= 0 &&
                    static_cast<std::size_t>(group.brush) < c.brushes.size())
                mesh.lightmapImage = lightmapImageFor(c, out, map,
                        c.brushes[static_cast<std::size_t>(group.brush)].lightmap);
            primitive.attributeMask = kVertexPosition;
            if (parsed.hasNormal) primitive.attributeMask |= kVertexNormal;
            if (parsed.hasColor) primitive.attributeMask |= kVertexColor;
            if (parsed.hasUv) primitive.attributeMask |= kVertexUv;
            if (parsed.hasUv2) primitive.attributeMask |= kVertexUv2;
            if (parsed.skinned) primitive.attributeMask |= kVertexJoints | kVertexWeights;
            primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
            primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
            primitive.sourceBytes = surfaceBytes(surface);
            primitive.source = static_cast<std::uint32_t>(owned->surfaces.size());
            primitive.aabb = MeshStore::computeAabb(surface.positions);
            primitive.hasAabb = true;
            primitive.skinned = parsed.skinned;

            out.geometryBytes += primitive.sourceBytes;
            ++out.primitiveCount;
            mesh.primitives.push_back(primitive);
            owned->surfaces.push_back(static_cast<SurfaceData&&>(surface));
        }
        out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));
        node.mesh = static_cast<int>(out.meshes.size() - 1);
        if (parsed.skinned) node.skeleton = 0;
    }

    if (c.sawBones)
    {
        Skeleton skeleton;
        if (!buildSkeleton(c, skeleton)) return fail(out, "skeleton is invalid");
        out.skeletons.push_back(static_cast<Skeleton&&>(skeleton));

        if (c.sawAnim && c.animFrames > 1)
        {
            const float lastFrame = static_cast<float>(c.animFrames - 1);
            ct::Vector<Sequence> clips;
            for (std::size_t s = 0; s < sequences.size(); ++s)
            {
                Sequence sequence = sequences[s];
                if (sequence.first > lastFrame) continue;
                if (sequence.last > lastFrame) sequence.last = lastFrame;
                clips.push_back(static_cast<Sequence&&>(sequence));
            }
            if (clips.empty())
            {
                Sequence whole;
                whole.name = baseName;
                whole.first = 0.0f;
                whole.last = lastFrame;
                clips.push_back(static_cast<Sequence&&>(whole));
            }
            for (std::size_t s = 0; s < clips.size(); ++s)
            {
                AnimationClip clip;
                clip.setName(clips[s].name);
                clip.setSkeleton(0);
                clip.setDuration((clips[s].last - clips[s].first) / c.animFps);
                for (std::size_t i = 0; i < c.nodes.size(); ++i)
                {
                    BoneTrack track;
                    if (buildTrack(static_cast<int>(i), c.nodes[i], c.animFps, clips[s].first,
                                clips[s].last, track))
                        clip.tracks().push_back(static_cast<BoneTrack&&>(track));
                }
                if (!clip.tracks().empty())
                    out.animations.push_back(static_cast<AnimationClip&&>(clip));
            }
        }
    }

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<B3DSource>&&>(owned));
    return true;
}

} // namespace

const char* B3DImporter::name() const
{
    return "Blitz3D";
}

bool B3DImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "b3d") == 0;
}

bool B3DImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    ct::Vector<Sequence> sequences;
    const ct::String sidecar = sequencePath(path);
    if (FileSystem::getSingleton().exists(sidecar))
        parseSequences(FileSystem::getSingleton().readText(sidecar), sequences);
    return build(bytes.data(), bytes.size(), path, sequences, out);
}

bool B3DImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty model data");
    return build(data, size, virtualName, ct::Vector<Sequence>(), out);
}

bool B3DImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const B3DSource* source = static_cast<const B3DSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool B3DImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
