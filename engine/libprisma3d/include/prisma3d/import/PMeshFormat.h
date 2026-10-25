#pragma once

#include <cstddef>
#include <cstdint>

namespace prisma3d
{
namespace pmesh
{

constexpr std::uint32_t fourcc(char a, char b, char c, char d)
{
    return static_cast<std::uint32_t>(static_cast<unsigned char>(a)) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(b)) << 8) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(c)) << 16) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(d)) << 24);
}

constexpr std::uint32_t kMeshMagic = fourcc('P', 'M', 'S', 'H');
constexpr std::uint32_t kSkeletonMagic = fourcc('P', 'S', 'K', 'L');
constexpr std::uint32_t kAnimationMagic = fourcc('P', 'A', 'N', 'M');

constexpr std::uint16_t kVersionMajor = 1;
constexpr std::uint16_t kVersionMinor = 0;
constexpr std::uint32_t kVersion =
        (static_cast<std::uint32_t>(kVersionMajor) << 16) | kVersionMinor;

constexpr const char* kMeshExtension = "pmesh";
constexpr const char* kSkeletonExtension = "pskel";
constexpr const char* kAnimationExtension = "panim";

constexpr std::size_t kHeaderSize = 12;
constexpr std::size_t kChunkHeaderSize = 12;
constexpr std::uint32_t kMaxString = 1u << 20;
constexpr std::uint32_t kMaxVertices = 100000000u;
constexpr std::uint32_t kMaxIndices = 300000000u;
constexpr std::uint32_t kMaxSurfaces = 1u << 20;
constexpr std::uint32_t kMaxMeshes = 1u << 20;
constexpr std::uint32_t kMaxSubMeshes = 1u << 20;
constexpr std::uint32_t kMaxNodes = 1u << 24;
constexpr std::uint32_t kMaxBones = 65535;
constexpr std::uint32_t kMaxClips = 1u << 20;
constexpr std::uint32_t kMaxKeys = 100000000u;
constexpr std::uint32_t kMaxTracks = 1u << 24;
constexpr std::uint32_t kMaxLinks = 65536;

constexpr std::uint32_t kNoSlot = 0xFFFFFFFFu;

enum HeaderFlags : std::uint32_t
{
    LittleEndian = 1u << 0,
    Skinned = 1u << 1
};

constexpr std::uint32_t kHeaderAttribShift = 4;

enum Chunk : std::uint32_t
{
    MaterialSlots = fourcc('M', 'A', 'T', 'S'),
    VertexBuffer = fourcc('V', 'B', 'U', 'F'),
    IndexBuffer = fourcc('I', 'B', 'U', 'F'),
    SubMeshes = fourcc('S', 'U', 'B', 'M'),
    Skin = fourcc('S', 'K', 'I', 'N'),
    Nodes = fourcc('N', 'O', 'D', 'E'),
    Cameras = fourcc('C', 'A', 'M', 'S'),
    ShadowMesh = fourcc('S', 'H', 'D', 'W'),
    SkeletonLink = fourcc('S', 'L', 'N', 'K'),
    AnimationLink = fourcc('A', 'L', 'N', 'K'),
    Bones = fourcc('B', 'O', 'N', 'E'),
    Clip = fourcc('C', 'L', 'I', 'P'),
    End = fourcc('E', 'N', 'D', 'C')
};

enum VertexAttribFlags : std::uint32_t
{
    HasNormal = 1u << 0,
    HasTangent = 1u << 1,
    HasUV = 1u << 2,
    HasUV2 = 1u << 3,
    HasColor = 1u << 4
};

constexpr std::uint32_t kKnownAttribs = HasNormal | HasTangent | HasUV | HasUV2 | HasColor;

constexpr std::uint32_t vertexAttribStride(std::uint32_t attribFlags)
{
    std::uint32_t stride = 12;
    if (attribFlags & HasNormal) stride += 12;
    if (attribFlags & HasTangent) stride += 16;
    if (attribFlags & HasUV) stride += 8;
    if (attribFlags & HasUV2) stride += 8;
    if (attribFlags & HasColor) stride += 16;
    return stride;
}

constexpr std::uint32_t kSkinVertexSize = 20;

enum IndexFlags : std::uint32_t
{
    Index16 = 1u << 0
};

enum SubMeshFlags : std::uint32_t
{
    SubMeshSkinned = 1u << 0,
    SubMeshHasBounds = 1u << 1
};

struct VertexBufferHeader
{
    std::uint32_t surface = 0;
    std::uint32_t attribs = 0;
    std::uint32_t vertexCount = 0;
    std::uint32_t topology = 0;
};
constexpr std::size_t kVertexBufferHeaderSize = 16;

struct IndexBufferHeader
{
    std::uint32_t surface = 0;
    std::uint32_t flags = 0;
    std::uint32_t indexCount = 0;
};
constexpr std::size_t kIndexBufferHeaderSize = 12;

struct SkinHeader
{
    std::uint32_t surface = 0;
    std::uint32_t vertexCount = 0;
};
constexpr std::size_t kSkinHeaderSize = 8;

struct SubMeshRecord
{
    std::uint32_t surface = 0;
    std::uint32_t materialSlot = kNoSlot;
    std::uint32_t flags = 0;
    float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
    float boundsMax[3] = { 0.0f, 0.0f, 0.0f };
};
constexpr std::size_t kSubMeshRecordSize = 36;

constexpr std::size_t kNodeFixedSize = 4 + 4 * 4 + 40 + 4;
constexpr std::size_t kCameraRecordSize = 28;
constexpr std::size_t kShadowSurfaceHeaderSize = 16;
constexpr std::size_t kBoneFixedSize = 4 + 4 + 128;

constexpr std::uint32_t kLoopModeCount = 3;

} // namespace pmesh
} // namespace prisma3d
