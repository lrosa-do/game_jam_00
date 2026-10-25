#include "prisma3d/render/ClusterBuilder.h"
#include "prisma3d/render/ShaderVariants.h"


#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint32_t kSphereVertexCount = 42;
const float kSphereVertices[kSphereVertexCount * 3] = { 0.0f, 0.0f, -1.0f, 0.7236073f, -0.5257253f,
    -0.4472195f, -0.276388f, -0.8506492f, -0.4472199f, -0.8944262f, 0.0f, -0.4472156f, -0.276388f,
    0.8506492f, -0.4472199f, 0.7236073f, 0.5257253f, -0.4472195f, 0.276388f, -0.8506492f,
    0.4472199f, -0.7236073f, -0.5257253f, 0.4472195f, -0.7236073f, 0.5257253f, 0.4472195f,
    0.276388f, 0.8506492f, 0.4472199f, 0.8944262f, 0.0f, 0.4472156f, 0.0f, 0.0f, 1.0f, -0.1624555f,
    -0.4999952f, -0.8506544f, 0.4253227f, -0.3090114f, -0.8506542f, 0.2628688f, -0.8090116f,
    -0.5257377f, 0.8506479f, 0.0f, -0.5257359f, 0.4253227f, 0.3090114f, -0.8506542f, -0.5257298f,
    0.0f, -0.8506517f, -0.6881894f, -0.4999969f, -0.5257362f, -0.1624555f, 0.4999952f, -0.8506544f,
    -0.6881894f, 0.4999969f, -0.5257362f, 0.2628688f, 0.8090116f, -0.5257377f, 0.9510579f,
    -0.3090126f, 0.0f, 0.9510579f, 0.3090126f, 0.0f, 0.0f, -1.0f, 0.0f, 0.5877856f, -0.8090167f,
    0.0f, -0.9510579f, -0.3090126f, 0.0f, -0.5877856f, -0.8090167f, 0.0f, -0.5877856f, 0.8090167f,
    0.0f, -0.9510579f, 0.3090126f, 0.0f, 0.5877856f, 0.8090167f, 0.0f, 0.0f, 1.0f, 0.0f, 0.6881894f,
    -0.4999969f, 0.5257362f, -0.2628688f, -0.8090116f, 0.5257377f, -0.8506479f, 0.0f, 0.5257359f,
    -0.2628688f, 0.8090116f, 0.5257377f, 0.6881894f, 0.4999969f, 0.5257362f, 0.1624555f,
    -0.4999952f, 0.8506544f, 0.5257298f, 0.0f, 0.8506517f, -0.4253227f, -0.3090114f, 0.8506542f,
    -0.4253227f, 0.3090114f, 0.8506542f, 0.1624555f, 0.4999952f, 0.8506544f };
const std::uint32_t kSphereTriangleCount = 80;
const std::uint16_t kSphereIndices[kSphereTriangleCount * 3] = { 0, 13, 12, 1, 13, 15, 0, 12, 17, 0,
    17, 19, 0, 19, 16, 1, 15, 22, 2, 14, 24, 3, 18, 26, 4, 20, 28, 5, 21, 30, 1, 22, 25, 2, 24, 27,
    3, 26, 29, 4, 28, 31, 5, 30, 23, 6, 32, 37, 7, 33, 39, 8, 34, 40, 9, 35, 41, 10, 36, 38, 38, 41,
    11, 38, 36, 41, 36, 9, 41, 41, 40, 11, 41, 35, 40, 35, 8, 40, 40, 39, 11, 40, 34, 39, 34, 7, 39,
    39, 37, 11, 39, 33, 37, 33, 6, 37, 37, 38, 11, 37, 32, 38, 32, 10, 38, 23, 36, 10, 23, 30, 36,
    30, 9, 36, 31, 35, 9, 31, 28, 35, 28, 8, 35, 29, 34, 8, 29, 26, 34, 26, 7, 34, 27, 33, 7, 27,
    24, 33, 24, 6, 33, 25, 32, 6, 25, 22, 32, 22, 10, 32, 30, 31, 9, 30, 21, 31, 21, 4, 31, 28, 29,
    8, 28, 20, 29, 20, 3, 29, 26, 27, 7, 26, 18, 27, 18, 2, 27, 24, 25, 6, 24, 14, 25, 14, 1, 25,
    22, 23, 10, 22, 15, 23, 15, 5, 23, 16, 21, 5, 16, 19, 21, 19, 4, 21, 19, 20, 4, 19, 17, 20, 17,
    3, 20, 17, 18, 3, 17, 12, 18, 12, 2, 18, 15, 16, 5, 15, 13, 16, 13, 0, 16, 12, 14, 2, 12, 13,
    14, 13, 1, 14 };

const std::uint32_t kConeVertexCount = 99;
const float kConeVertices[kConeVertexCount * 3] = { 0.0f, 1.0f, -1.0f, 0.1950903f, 0.9807853f,
    -1.0f, 0.3826835f, 0.9238795f, -1.0f, 0.5555703f, 0.8314696f, -1.0f, 0.7071068f, 0.7071068f,
    -1.0f, 0.8314697f, 0.5555702f, -1.0f, 0.9238795f, 0.3826834f, -1.0f, 0.9807853f, 0.1950903f,
    -1.0f, 1.0f, 0.0f, -1.0f, 0.9807853f, -0.1950902f, -1.0f, 0.9238796f, -0.3826833f, -1.0f,
    0.8314697f, -0.5555702f, -1.0f, 0.7071068f, -0.7071068f, -1.0f, 0.5555702f, -0.8314697f, -1.0f,
    0.3826833f, -0.9238796f, -1.0f, 0.1950901f, -0.9807853f, -1.0f, -3.25841e-7f, -1.0f, -1.0f,
    -0.1950907f, -0.9807852f, -1.0f, -0.3826839f, -0.9238793f, -1.0f, -0.5555707f, -0.8314693f,
    -1.0f, -0.7071073f, -0.7071063f, -1.0f, -0.83147f, -0.5555697f, -1.0f, -0.9238799f, -0.3826827f,
    -1.0f, 0.0f, 0.0f, 0.0f, -0.9807854f, -0.1950894f, -1.0f, -1.0f, 9.65599e-7f, -1.0f,
    -0.9807851f, 0.1950913f, -1.0f, -0.9238791f, 0.3826845f, -1.0f, -0.8314689f, 0.5555713f, -1.0f,
    -0.7071059f, 0.7071077f, -1.0f, -0.5555691f, 0.8314704f, -1.0f, -0.3826821f, 0.9238801f, -1.0f,
    -0.1950888f, 0.9807856f, -1.0f };
const std::uint32_t kConeTriangleCount = 62;
const std::uint16_t kConeIndices[kConeTriangleCount * 3] = { 0, 23, 1, 1, 23, 2, 2, 23, 3, 3, 23, 4,
    4, 23, 5, 5, 23, 6, 6, 23, 7, 7, 23, 8, 8, 23, 9, 9, 23, 10, 10, 23, 11, 11, 23, 12, 12, 23, 13,
    13, 23, 14, 14, 23, 15, 15, 23, 16, 16, 23, 17, 17, 23, 18, 18, 23, 19, 19, 23, 20, 20, 23, 21,
    21, 23, 22, 22, 23, 24, 24, 23, 25, 25, 23, 26, 26, 23, 27, 27, 23, 28, 28, 23, 29, 29, 23, 30,
    30, 23, 31, 31, 23, 32, 32, 23, 0, 7, 15, 24, 32, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 3, 6, 7, 3, 7,
    8, 9, 9, 10, 7, 10, 11, 7, 11, 12, 15, 12, 13, 15, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 20,
    24, 20, 21, 24, 21, 22, 24, 24, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 32, 1, 3, 15, 17,
    24, 17, 19, 24, 24, 26, 32, 26, 28, 32, 28, 30, 32, 32, 3, 7, 7, 11, 15, 32, 7, 24 };

const std::uint32_t kBoxVertexCount = 8;
const float kBoxVertices[kBoxVertexCount * 3] = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, -1.0f,
    1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f, 1.0f, -1.0f, 1.0f,
    1.0f, 1.0f };
const std::uint32_t kBoxTriangleCount = 12;
const std::uint16_t kBoxIndices[kBoxTriangleCount * 3] = { 1, 2, 0, 3, 6, 2, 7, 4, 6, 5, 0, 4, 6, 0,
    2, 3, 5, 7, 1, 3, 2, 3, 7, 6, 7, 5, 4, 5, 1, 0, 6, 4, 0, 3, 1, 5 };

const std::uint32_t kMsaaSamples = 4;
const std::uint32_t kClearGroupSize = 256;
const std::uint32_t kClearRowGroups = 4096;

std::uint32_t alignValue(std::uint32_t value, std::uint32_t alignment)
{
    if (alignment <= 1) return value;
    return (value + alignment - 1) / alignment * alignment;
}

std::uint32_t shiftFromPowerOf2(std::uint32_t value)
{
    for (std::uint32_t i = 0; i < 32; ++i)
        if (value == (1u << i)) return i;
    return 0;
}

void clearGroups(std::uint32_t words, std::uint32_t& groupsX, std::uint32_t& groupsY)
{
    const std::uint32_t groups = (words + kClearGroupSize - 1) / kClearGroupSize;
    groupsX = groups < kClearRowGroups ? groups : kClearRowGroups;
    if (groupsX == 0) groupsX = 1;
    groupsY = (groups + groupsX - 1) / groupsX;
    if (groupsY == 0) groupsY = 1;
}

struct Xform
{
    Math::Vec3 rows[3];
    Math::Vec3 origin;
};

Xform toXform(const Math::Mat4& matrix)
{
    Xform result;
    for (int i = 0; i < 3; ++i)
        result.rows[i] = Math::Vec3(matrix[0].v[i], matrix[1].v[i], matrix[2].v[i]);
    result.origin = Math::Vec3(matrix[3].v[0], matrix[3].v[1], matrix[3].v[2]);
    return result;
}

Math::Vec3 column(const Xform& x, int c)
{
    return Math::Vec3(x.rows[0][c], x.rows[1][c], x.rows[2][c]);
}

void setColumn(Xform& x, int c, const Math::Vec3& v)
{
    x.rows[0][c] = v.x;
    x.rows[1][c] = v.y;
    x.rows[2][c] = v.z;
}

Math::Vec3 normalizedOrZero(const Math::Vec3& v)
{
    const float lengthSquared = v.LengthSquared();
    if (lengthSquared == 0.0f) return Math::Vec3(0.0f, 0.0f, 0.0f);
    return v / sqrtf(lengthSquared);
}

Math::Vec3 absolute(const Math::Vec3& v)
{
    return Math::Vec3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

float uniformScale(const Xform& x)
{
    return (x.rows[0].Length() + x.rows[1].Length() + x.rows[2].Length()) / 3.0f;
}

void orthonormalize(Xform& x)
{
    Math::Vec3 a = column(x, 0);
    Math::Vec3 b = column(x, 1);
    Math::Vec3 c = column(x, 2);
    a = normalizedOrZero(a);
    b = b - a * a.Dot(b);
    b = normalizedOrZero(b);
    c = c - a * a.Dot(c) - b * b.Dot(c);
    c = normalizedOrZero(c);
    setColumn(x, 0, a);
    setColumn(x, 1, b);
    setColumn(x, 2, c);
}

Math::Vec3 transformPoint(const Xform& x, const Math::Vec3& v)
{
    return Math::Vec3(x.rows[0].Dot(v) + x.origin.x, x.rows[1].Dot(v) + x.origin.y,
            x.rows[2].Dot(v) + x.origin.z);
}

Math::Vec3 basisTransformInverse(const Xform& x, const Math::Vec3& v)
{
    return Math::Vec3(x.rows[0].x * v.x + x.rows[1].x * v.y + x.rows[2].x * v.z,
            x.rows[0].y * v.x + x.rows[1].y * v.y + x.rows[2].y * v.z,
            x.rows[0].z * v.x + x.rows[1].z * v.y + x.rows[2].z * v.z);
}

Math::Vec3 transformPointInverse(const Xform& x, const Math::Vec3& v)
{
    return basisTransformInverse(x, v - x.origin);
}

void storeTransposed3x4(const Xform& x, float* out)
{
    for (int i = 0; i < 3; ++i)
    {
        out[i * 4 + 0] = x.rows[i].x;
        out[i * 4 + 1] = x.rows[i].y;
        out[i * 4 + 2] = x.rows[i].z;
        out[i * 4 + 3] = x.origin[i];
    }
}

float planeDistance(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    Math::Vec3 normal = (a - c).Cross(a - b);
    normal = normalizedOrZero(normal);
    return normal.Dot(a);
}

float arcCosine(float x)
{
    return x < -1.0f ? Math::PI : (x > 1.0f ? 0.0f : acosf(x));
}

float zNearOf(const Math::Mat4& m)
{
    return (m.col3.w + m.col3.z) / (m.col2.w + m.col2.z);
}

float zFarOf(const Math::Mat4& m)
{
    return (m.col3.w - m.col3.z) / (m.col2.w - m.col2.z);
}

} // namespace

ClusterBuilderShared::ClusterBuilderShared()
        : mDriver(nullptr),
          mSphereOverfit(0.0f),
          mConeOverfit(0.0f)
{
}

ClusterBuilderShared::~ClusterBuilderShared()
{
    destroy();
}

bool ClusterBuilderShared::ready() const
{
    return mDriver != nullptr && mRenderPipeline.valid() && mStorePipeline.valid() &&
           mClearPipeline.valid() && mSphere.vertexBuffer.valid() && mCone.vertexBuffer.valid() &&
           mBox.vertexBuffer.valid();
}

prisma::Driver* ClusterBuilderShared::driver() const
{
    return mDriver;
}

float ClusterBuilderShared::sphereOverfit() const
{
    return mSphereOverfit;
}

float ClusterBuilderShared::coneOverfit() const
{
    return mConeOverfit;
}

bool ClusterBuilderShared::createMesh(Mesh& mesh, const float* vertices, std::uint32_t vertexCount,
        const std::uint16_t* indices, std::uint32_t indexCount, const char* name)
{
    prisma::BufferDesc vertexDesc;
    vertexDesc.usage = prisma::BufferUsage::Vertex;
    vertexDesc.size = static_cast<std::uint32_t>(sizeof(float)) * vertexCount * 3;
    vertexDesc.data = vertices;
    vertexDesc.debugName = name;
    mesh.vertexBuffer = mDriver->createBuffer(vertexDesc);

    prisma::BufferDesc indexDesc;
    indexDesc.usage = prisma::BufferUsage::Index;
    indexDesc.size = static_cast<std::uint32_t>(sizeof(std::uint16_t)) * indexCount;
    indexDesc.data = indices;
    indexDesc.indexFormat = prisma::IndexFormat::UInt16;
    indexDesc.debugName = name;
    mesh.indexBuffer = mDriver->createBuffer(indexDesc);
    mesh.indexCount = indexCount;
    return mesh.vertexBuffer.valid() && mesh.indexBuffer.valid();
}

bool ClusterBuilderShared::create(prisma::Driver* driver)
{
    if (!driver) return false;
    destroy();
    const prisma::Caps& caps = driver->caps();
    if (!caps.compute || !caps.storageBuffersInGraphics || !caps.storageWritesInGraphics ||
            caps.maxSamples < kMsaaSamples)
    {
        log_error("render: the cluster builder needs compute, storage writes in fragment shaders "
                  "and 4 samples");
        return false;
    }
    mDriver = driver;

    bool created = createMesh(mSphere, kSphereVertices, kSphereVertexCount, kSphereIndices,
            kSphereTriangleCount * 3, "cluster sphere");
    created = createMesh(mCone, kConeVertices, kConeVertexCount, kConeIndices,
                      kConeTriangleCount * 3, "cluster cone") &&
              created;
    created = createMesh(mBox, kBoxVertices, kBoxVertexCount, kBoxIndices, kBoxTriangleCount * 3,
                      "cluster box") &&
              created;

    float minimum = 1e20f;
    for (std::uint32_t i = 0; i < kSphereTriangleCount; ++i)
    {
        Math::Vec3 vertices[3];
        for (std::uint32_t j = 0; j < 3; ++j)
        {
            const std::uint32_t index = kSphereIndices[i * 3 + j];
            for (std::uint32_t k = 0; k < 3; ++k) vertices[j][k] = kSphereVertices[index * 3 + k];
        }
        const float d = fabsf(planeDistance(vertices[0], vertices[1], vertices[2]));
        if (d < minimum) minimum = d;
    }
    mSphereOverfit = 1.0f / minimum;

    minimum = 1e20f;
    for (std::uint32_t i = 0; i < kConeTriangleCount; ++i)
    {
        Math::Vec3 vertices[3];
        int zeroIndex = -1;
        for (std::uint32_t j = 0; j < 3; ++j)
        {
            const std::uint32_t index = kConeIndices[i * 3 + j];
            for (std::uint32_t k = 0; k < 3; ++k) vertices[j][k] = kConeVertices[index * 3 + k];
            if (vertices[j] == Math::Vec3(0.0f, 0.0f, 0.0f)) zeroIndex = static_cast<int>(j);
        }
        if (zeroIndex != -1)
        {
            const Math::Vec3 a = vertices[(zeroIndex + 1) % 3];
            const Math::Vec3 b = vertices[(zeroIndex + 2) % 3];
            const Math::Vec3 c = a + Math::Vec3(0.0f, 0.0f, 1.0f);
            const float d = fabsf(planeDistance(a, b, c));
            if (d < minimum) minimum = d;
        }
    }
    mConeOverfit = 1.0f / minimum;

    mRenderVertexShader = driver->createShader(prisma3d::shaderDesc(cluster_render_vert, caps));
    mRenderFragmentShader = driver->createShader(prisma3d::shaderDesc(cluster_render_frag, caps));
    mStoreShader = driver->createShader(prisma3d::shaderDesc(cluster_store_comp, caps));
    mClearShader = driver->createShader(prisma3d::shaderDesc(cluster_clear_comp, caps));
    created = created && mRenderVertexShader.valid() && mRenderFragmentShader.valid() &&
              mStoreShader.valid() && mClearShader.valid();

    if (created)
    {
        prisma::PipelineDesc desc;
        desc.vertexShader = mRenderVertexShader;
        desc.fragmentShader = mRenderFragmentShader;
        desc.vertexBuffers[0].stride = static_cast<std::uint32_t>(sizeof(float)) * 3;
        desc.vertexBufferCount = 1;
        desc.attributes[0].location = 0;
        desc.attributes[0].format = prisma::VertexFormat::Float3;
        desc.attributes[0].offset = 0;
        desc.attributes[0].buffer = 0;
        desc.attributeCount = 1;
        desc.topology = prisma::Topology::Triangles;
        desc.targets.window = false;
        desc.targets.samples = kMsaaSamples;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA8;
        desc.targets.colorCount = 1;
        desc.uniformBlocks[0].name = "Params";
        desc.uniformBlocks[0].slot = 0;
        desc.uniformBlocks[1].name = "State";
        desc.uniformBlocks[1].slot = 1;
        desc.uniformBlockCount = 2;
        desc.storageBuffers[0].name = "RenderElements";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "ClusterRender";
        desc.storageBuffers[1].slot = 1;
        desc.storageBufferCount = 2;
        desc.depthTest = false;
        desc.depthWrite = false;
        desc.cullMode = prisma::CullMode::None;
        desc.blend = true;
        desc.srcColor = prisma::BlendFactor::SrcAlpha;
        desc.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
        desc.srcAlpha = prisma::BlendFactor::SrcAlpha;
        desc.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
        desc.debugName = "cluster render";
        mRenderPipeline = driver->createPipeline(desc);

        prisma::ComputePipelineDesc store;
        store.shader = mStoreShader;
        store.uniformBlocks[0].name = "Params";
        store.uniformBlocks[0].slot = 0;
        store.uniformBlockCount = 1;
        store.storageBuffers[0].name = "ClusterRender";
        store.storageBuffers[0].slot = 0;
        store.storageBuffers[1].name = "ClusterStore";
        store.storageBuffers[1].slot = 1;
        store.storageBuffers[2].name = "RenderElements";
        store.storageBuffers[2].slot = 2;
        store.storageBufferCount = 3;
        store.debugName = "cluster store";
        mStorePipeline = driver->createComputePipeline(store);

        prisma::ComputePipelineDesc clear;
        clear.shader = mClearShader;
        clear.uniformBlocks[0].name = "Params";
        clear.uniformBlocks[0].slot = 0;
        clear.uniformBlockCount = 1;
        clear.storageBuffers[0].name = "ClusterClear";
        clear.storageBuffers[0].slot = 0;
        clear.storageBufferCount = 1;
        clear.debugName = "cluster clear";
        mClearPipeline = driver->createComputePipeline(clear);
    }

    if (!ready())
    {
        log_error("render: could not create the cluster builder resources");
        destroy();
        return false;
    }
    return true;
}

void ClusterBuilderShared::destroy()
{
    if (!mDriver) return;
    if (mRenderPipeline.valid()) mDriver->destroy(mRenderPipeline);
    if (mStorePipeline.valid()) mDriver->destroy(mStorePipeline);
    if (mClearPipeline.valid()) mDriver->destroy(mClearPipeline);
    if (mRenderVertexShader.valid()) mDriver->destroy(mRenderVertexShader);
    if (mRenderFragmentShader.valid()) mDriver->destroy(mRenderFragmentShader);
    if (mStoreShader.valid()) mDriver->destroy(mStoreShader);
    if (mClearShader.valid()) mDriver->destroy(mClearShader);
    Mesh* meshes[3] = { &mSphere, &mCone, &mBox };
    for (Mesh* mesh: meshes)
    {
        if (mesh->vertexBuffer.valid()) mDriver->destroy(mesh->vertexBuffer);
        if (mesh->indexBuffer.valid()) mDriver->destroy(mesh->indexBuffer);
        *mesh = Mesh();
    }
    mRenderPipeline = prisma::PipelineHandle();
    mStorePipeline = prisma::PipelineHandle();
    mClearPipeline = prisma::PipelineHandle();
    mRenderVertexShader = prisma::ShaderHandle();
    mRenderFragmentShader = prisma::ShaderHandle();
    mStoreShader = prisma::ShaderHandle();
    mClearShader = prisma::ShaderHandle();
    mSphereOverfit = 0.0f;
    mConeOverfit = 0.0f;
    mDriver = nullptr;
}

ClusterBuilder::ClusterBuilder()
        : mShared(nullptr),
          mDriver(nullptr),
          mMaxElementsByType(0),
          mRenderElementCount(0),
          mRenderElementMax(0),
          mZFar(0.0f),
          mZNear(0.0f),
          mCameraOrthogonal(false),
          mClusterSize(32),
          mDivisor(2),
          mScreenWidth(0),
          mScreenHeight(0),
          mClusterWidth(0),
          mClusterHeight(0),
          mClusterRenderBufferSize(0),
          mClusterBufferSize(0),
          mUniformStride(0)
{
    for (std::uint32_t i = 0; i < kElementTypeMax; ++i) mClusterCountByType[i] = 0;
}

ClusterBuilder::~ClusterBuilder()
{
    destroy();
}

void ClusterBuilder::setShared(ClusterBuilderShared* shared)
{
    mShared = shared;
}

void ClusterBuilder::destroy()
{
    if (mDriver)
    {
        if (mTarget.valid()) mDriver->destroy(mTarget);
        if (mClusterRenderBuffer.valid()) mDriver->destroy(mClusterRenderBuffer);
        if (mClusterBuffer.valid()) mDriver->destroy(mClusterBuffer);
        if (mElementBuffer.valid()) mDriver->destroy(mElementBuffer);
        if (mStateBuffer.valid()) mDriver->destroy(mStateBuffer);
        if (mDrawParamsBuffer.valid()) mDriver->destroy(mDrawParamsBuffer);
        if (mStoreParamsBuffer.valid()) mDriver->destroy(mStoreParamsBuffer);
        if (mClearParamsBuffer.valid()) mDriver->destroy(mClearParamsBuffer);
    }
    mTarget = prisma::TextureHandle();
    mClusterRenderBuffer = prisma::BufferHandle();
    mClusterBuffer = prisma::BufferHandle();
    mElementBuffer = prisma::BufferHandle();
    mStateBuffer = prisma::BufferHandle();
    mDrawParamsBuffer = prisma::BufferHandle();
    mStoreParamsBuffer = prisma::BufferHandle();
    mClearParamsBuffer = prisma::BufferHandle();
    mRenderElements.clear();
    mDrawParamsData.clear();
    mRenderElementCount = 0;
    mRenderElementMax = 0;
    mMaxElementsByType = 0;
    mClusterRenderBufferSize = 0;
    mClusterBufferSize = 0;
    mScreenWidth = 0;
    mScreenHeight = 0;
    mClusterWidth = 0;
    mClusterHeight = 0;
    for (std::uint32_t i = 0; i < kElementTypeMax; ++i) mClusterCountByType[i] = 0;
    mDriver = nullptr;
}

bool ClusterBuilder::setup(std::uint32_t screenWidth, std::uint32_t screenHeight,
        std::uint32_t maxElements)
{
    destroy();
    if (!mShared || !mShared->ready() || maxElements == 0 || screenWidth < 1 || screenHeight < 1)
        return false;
    mDriver = mShared->driver();
    const prisma::Caps& caps = mDriver->caps();

    mScreenWidth = screenWidth;
    mScreenHeight = screenHeight;
    mClusterWidth = (screenWidth + mClusterSize - 1) / mClusterSize;
    mClusterHeight = (screenHeight + mClusterSize - 1) / mClusterSize;

    mMaxElementsByType = maxElements;
    if (mMaxElementsByType % 32) mMaxElementsByType += 32 - (mMaxElementsByType % 32);

    const std::uint64_t cells = static_cast<std::uint64_t>(mClusterWidth) * mClusterHeight;
    const std::uint64_t renderElementMax =
            static_cast<std::uint64_t>(mMaxElementsByType) * kElementTypeMax;
    const std::uint64_t clusterBytes = cells * (mMaxElementsByType / 32 + 32) * kElementTypeMax * 4;
    const std::uint64_t renderBytes = cells * (renderElementMax / 32 + renderElementMax) * 4;
    if (clusterBytes > 0xFFFFFFFFull || renderBytes > 0xFFFFFFFFull || mMaxElementsByType > 0xFFFFu)
    {
        log_error("render: the cluster buffers for this screen size and element count are too "
                  "large");
        destroy();
        return false;
    }
    mClusterBufferSize = static_cast<std::uint32_t>(clusterBytes);
    mClusterRenderBufferSize = static_cast<std::uint32_t>(renderBytes);
    mRenderElementMax = static_cast<std::uint32_t>(renderElementMax);
    mRenderElementCount = 0;
    mRenderElements.resize(mRenderElementMax);
    mUniformStride = alignValue(32, caps.uniformBufferOffsetAlignment);

    prisma::BufferDesc desc;
    desc.usage = prisma::BufferUsage::Storage;
    desc.update = prisma::BufferUpdate::Static;
    desc.size = mClusterRenderBufferSize;
    desc.debugName = "cluster render";
    mClusterRenderBuffer = mDriver->createBuffer(desc);
    desc.size = mClusterBufferSize;
    desc.debugName = "cluster";
    mClusterBuffer = mDriver->createBuffer(desc);
    desc.update = prisma::BufferUpdate::Dynamic;
    desc.size = static_cast<std::uint32_t>(sizeof(RenderElementData)) * mRenderElementMax;
    desc.debugName = "cluster elements";
    mElementBuffer = mDriver->createBuffer(desc);

    desc.usage = prisma::BufferUsage::Uniform;
    desc.size = static_cast<std::uint32_t>(sizeof(StateUniform));
    desc.debugName = "cluster state";
    mStateBuffer = mDriver->createBuffer(desc);
    desc.size = mUniformStride * mRenderElementMax;
    desc.debugName = "cluster draws";
    mDrawParamsBuffer = mDriver->createBuffer(desc);
    desc.size = static_cast<std::uint32_t>(sizeof(StoreParams));
    desc.debugName = "cluster store params";
    mStoreParamsBuffer = mDriver->createBuffer(desc);

    ct::Vector<unsigned char> clearData;
    clearData.resize(static_cast<std::size_t>(mUniformStride) * 2);
    memset(clearData.data(), 0, clearData.size());
    std::uint32_t groupsX = 0;
    std::uint32_t groupsY = 0;
    ClearParams clearParams = {};
    clearParams.wordCount = mClusterBufferSize / 4;
    clearGroups(clearParams.wordCount, groupsX, groupsY);
    clearParams.rowWidth = groupsX * kClearGroupSize;
    memcpy(clearData.data(), &clearParams, sizeof(ClearParams));
    clearParams.wordCount = mClusterRenderBufferSize / 4;
    clearGroups(clearParams.wordCount, groupsX, groupsY);
    clearParams.rowWidth = groupsX * kClearGroupSize;
    memcpy(clearData.data() + mUniformStride, &clearParams, sizeof(ClearParams));
    desc.size = static_cast<std::uint32_t>(clearData.size());
    desc.data = clearData.data();
    desc.update = prisma::BufferUpdate::Static;
    desc.debugName = "cluster clear params";
    mClearParamsBuffer = mDriver->createBuffer(desc);

    const std::uint32_t divisor = 1u << mDivisor;
    prisma::TextureDesc target;
    target.type = prisma::TextureType::Texture2D;
    target.format = prisma::TextureFormat::RGBA8;
    target.width = screenWidth / divisor > 0 ? screenWidth / divisor : 1;
    target.height = screenHeight / divisor > 0 ? screenHeight / divisor : 1;
    target.samples = kMsaaSamples;
    target.usage = prisma::kTextureRenderTarget;
    target.debugName = "cluster target";
    mTarget = mDriver->createTexture(target);

    const bool created = mClusterRenderBuffer.valid() && mClusterBuffer.valid() &&
                         mElementBuffer.valid() && mStateBuffer.valid() &&
                         mDrawParamsBuffer.valid() && mStoreParamsBuffer.valid() &&
                         mClearParamsBuffer.valid() && mTarget.valid();
    if (!created)
    {
        log_error("render: could not create the cluster builder buffers");
        destroy();
        return false;
    }
    return true;
}

void ClusterBuilder::begin(const Math::Mat4& cameraTransform, const Math::Mat4& projection,
        bool flipY)
{
    mViewTransform = cameraTransform.InverseAffine();
    mProjection = projection;
    mZNear = zNearOf(mProjection);
    mZFar = zFarOf(mProjection);
    mCameraOrthogonal = projection.col2.w == 0.0f;
    mAdjustedProjection = mProjection;
    if (!mCameraOrthogonal)
    {
        const float zFar = zFarOf(mAdjustedProjection);
        const float zNear = 0.0001f;
        const float deltaZ = zFar - zNear;
        mAdjustedProjection.col2.z = -(zFar + zNear) / deltaZ;
        mAdjustedProjection.col3.z = -2.0f * zNear * zFar / deltaZ;
    }

    Math::Mat4 correction = Math::Mat4::Identity();
    correction.col1.y = flipY ? -1.0f : 1.0f;
    correction.col2.z = 0.5f;
    correction.col3.z = 0.5f;
    mProjection = correction * mProjection;
    mAdjustedProjection = correction * mAdjustedProjection;

    mRenderElementCount = 0;
    for (std::uint32_t i = 0; i < kElementTypeMax; ++i) mClusterCountByType[i] = 0;
}

bool ClusterBuilder::fits(ElementType type) const
{
    return !mRenderElements.empty() && mRenderElementCount < mRenderElementMax &&
           mClusterCountByType[type] < mMaxElementsByType;
}

void ClusterBuilder::addLight(LightType type, const Math::Mat4& transform, float radius,
        float spotAperture, const Math::Vec2& areaSize)
{
    if (type == kLightOmni && !fits(kElementOmniLight)) return;
    if (type == kLightSpot && !fits(kElementSpotLight)) return;
    if (type == kLightArea && !fits(kElementAreaLight)) return;

    RenderElementData& e = mRenderElements[mRenderElementCount];
    memset(&e, 0, sizeof(e));

    Xform xform = toXform(mViewTransform * transform);

    float scaledRadius = uniformScale(xform);
    if (scaledRadius < 0.98f || scaledRadius > 1.02f) orthonormalize(xform);

    scaledRadius *= radius;

    if (type == kLightOmni || (type == kLightSpot && spotAperture > kWideSpotAngleThresholdDeg))
    {
        scaledRadius *= mShared->mSphereOverfit;

        const float depth = -xform.origin.z;
        if (mCameraOrthogonal)
        {
            e.touchesNear = (depth - scaledRadius) < mZNear;
        }
        else
        {
            const float radius2 = scaledRadius * mShared->mSphereOverfit;
            e.touchesNear = xform.origin.LengthSquared() < radius2 * radius2;
        }

        e.touchesFar = (depth + scaledRadius) > mZFar;
        e.scale[0] = scaledRadius;
        e.scale[1] = scaledRadius;
        e.scale[2] = scaledRadius;
        if (type == kLightOmni)
        {
            e.type = kElementOmniLight;
            e.originalIndex = mClusterCountByType[kElementOmniLight];
            mClusterCountByType[kElementOmniLight]++;
        }
        else
        {
            e.type = kElementSpotLight;
            e.hasWideSpotAngle = 1;
            e.originalIndex = mClusterCountByType[kElementSpotLight];
            mClusterCountByType[kElementSpotLight]++;
        }

        storeTransposed3x4(xform, e.transformInv);
    }
    else if (type == kLightSpot)
    {
        scaledRadius *= mShared->mConeOverfit;

        const float len = tanf(spotAperture * Math::DEG2RAD) * scaledRadius;
        float maxD = -1e20f;
        float minD = 1e20f;
        const float corners[4][2] = { { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 } };
        for (const auto& corner: corners)
        {
            const float d = -transformPoint(xform,
                    Math::Vec3(len * corner[0], len * corner[1], -scaledRadius))
                                     .z;
            if (d < minD) minD = d;
            if (d > maxD) maxD = d;
        }

        if (mCameraOrthogonal)
        {
            e.touchesNear = minD < mZNear;
        }
        else
        {
            const Math::Vec3 axis = -column(xform, 2);
            const float dist = -(axis.Dot(xform.origin));
            if (dist >= 0.0f && dist < scaledRadius)
            {
                const float angle =
                        arcCosine((-normalizedOrZero(xform.origin)).Dot(axis)) * Math::RAD2DEG;
                e.touchesNear = angle < spotAperture * 1.05f;
            }
            else
            {
                e.touchesNear = false;
            }
        }

        e.touchesFar = maxD > mZFar;
        e.scale[0] = len * mShared->mConeOverfit;
        e.scale[1] = len * mShared->mConeOverfit;
        e.scale[2] = scaledRadius;
        e.hasWideSpotAngle = 0;
        e.type = kElementSpotLight;
        e.originalIndex = mClusterCountByType[kElementSpotLight];

        storeTransposed3x4(xform, e.transformInv);

        mClusterCountByType[kElementSpotLight]++;
    }
    else
    {
        Math::Vec3 scale(areaSize.x / 2.0f + scaledRadius, areaSize.y / 2.0f + scaledRadius,
                scaledRadius / 2.0f);

        for (int i = 0; i < 3; ++i)
        {
            const float s = xform.rows[i].Length();
            xform.rows[i] = xform.rows[i] / s;
        }
        xform.origin -= column(xform, 2) * scale.z;

        const float depth = -xform.origin.z;
        const float boxDepth = fabsf(basisTransformInverse(xform, Math::Vec3(0, 0, -1)).Dot(scale));

        if (mCameraOrthogonal)
        {
            e.touchesNear = (depth - boxDepth) < mZNear;
        }
        else
        {
            const Math::Vec3 inside = absolute(transformPointInverse(xform, Math::Vec3(0, 0, 0)));
            e.touchesNear = inside.x < scale.x && inside.y < scale.y && inside.z < scale.z;
        }

        e.touchesFar = depth + boxDepth > mZFar;

        e.scale[0] = scale.x;
        e.scale[1] = scale.y;
        e.scale[2] = scale.z;

        e.type = kElementAreaLight;
        e.originalIndex = mClusterCountByType[kElementAreaLight];

        storeTransposed3x4(xform, e.transformInv);

        mClusterCountByType[kElementAreaLight]++;
    }

    mRenderElementCount++;
}

void ClusterBuilder::addBox(BoxType type, const Math::Mat4& transform, const Math::Vec3& halfSize)
{
    const ElementType elementType = type == kBoxDecal ? kElementDecal : kElementReflectionProbe;
    if (!fits(elementType)) return;

    RenderElementData& e = mRenderElements[mRenderElementCount];
    memset(&e, 0, sizeof(e));
    Xform xform = toXform(mViewTransform * transform);

    Math::Vec3 scale = halfSize;
    for (int i = 0; i < 3; ++i)
    {
        const float s = xform.rows[i].Length();
        scale[i] *= s;
        xform.rows[i] = xform.rows[i] / s;
    }

    const float boxDepth = fabsf(basisTransformInverse(xform, Math::Vec3(0, 0, -1)).Dot(scale));
    const float depth = -xform.origin.z;

    if (mCameraOrthogonal)
    {
        e.touchesNear = depth - boxDepth < mZNear;
    }
    else
    {
        const Math::Vec3 inside = absolute(transformPointInverse(xform, Math::Vec3(0, 0, 0)));
        e.touchesNear = inside.x < scale.x && inside.y < scale.y && inside.z < scale.z;
    }

    e.touchesFar = depth + boxDepth > mZFar;

    e.scale[0] = scale.x;
    e.scale[1] = scale.y;
    e.scale[2] = scale.z;

    e.type = elementType;
    e.originalIndex = mClusterCountByType[e.type];

    storeTransposed3x4(xform, e.transformInv);

    mClusterCountByType[e.type]++;
    mRenderElementCount++;
}

void ClusterBuilder::dispatchClear(prisma::BufferHandle buffer, std::uint32_t bytes,
        std::uint32_t slot)
{
    std::uint32_t groupsX = 0;
    std::uint32_t groupsY = 0;
    clearGroups(bytes / 4, groupsX, groupsY);
    mDriver->bindUniformBuffer(0, mClearParamsBuffer, slot * mUniformStride,
            static_cast<std::uint32_t>(sizeof(ClearParams)));
    mDriver->bindStorageBuffer(0, buffer, 0, bytes);
    mDriver->dispatch(groupsX, groupsY, 1);
}

bool ClusterBuilder::bake()
{
    if (!mDriver || !mShared || !mShared->ready() || !mClusterBuffer.valid()) return false;

    mDriver->beginComputePass();
    mDriver->bindPipeline(mShared->mClearPipeline);
    dispatchClear(mClusterBuffer, mClusterBufferSize, 0);
    if (mRenderElementCount > 0) dispatchClear(mClusterRenderBuffer, mClusterRenderBufferSize, 1);
    mDriver->endComputePass();

    if (mRenderElementCount == 0) return true;

    StateUniform state = {};
    memcpy(state.projection, mAdjustedProjection.Data(), sizeof(state.projection));
    state.invZFar = 1.0f / mZFar;
    state.screenToClustersShift = shiftFromPowerOf2(mClusterSize) - mDivisor;
    state.clusterScreenWidth = mClusterWidth;
    state.clusterDepthOffset = mRenderElementMax / 32;
    state.clusterDataSize = state.clusterDepthOffset + mRenderElementMax;
    mDriver->updateBuffer(mStateBuffer, 0, &state, static_cast<std::uint32_t>(sizeof(state)));

    mDriver->updateBuffer(mElementBuffer, 0, mRenderElements.data(),
            static_cast<std::uint32_t>(sizeof(RenderElementData)) * mRenderElementCount);

    mDrawParamsData.resize(static_cast<std::size_t>(mUniformStride) * mRenderElementCount);
    memset(mDrawParamsData.data(), 0, mDrawParamsData.size());
    for (std::uint32_t i = 0; i < mRenderElementCount; ++i)
    {
        DrawParams params = {};
        params.baseIndex = i;
        memcpy(mDrawParamsData.data() + static_cast<std::size_t>(i) * mUniformStride, &params,
                sizeof(params));
    }
    mDriver->updateBuffer(mDrawParamsBuffer, 0, mDrawParamsData.data(),
            static_cast<std::uint32_t>(mDrawParamsData.size()));

    StoreParams store = {};
    store.clusterRenderDataSize = mRenderElementMax / 32 + mRenderElementMax;
    store.maxRenderElementCountDiv32 = mRenderElementMax / 32;
    store.clusterScreenSize[0] = mClusterWidth;
    store.clusterScreenSize[1] = mClusterHeight;
    store.renderElementCountDiv32 = (mRenderElementCount + 31) / 32;
    store.maxClusterElementCountDiv32 = mMaxElementsByType / 32;
    mDriver->updateBuffer(mStoreParamsBuffer, 0, &store, static_cast<std::uint32_t>(sizeof(store)));

    prisma::RenderPassDesc pass;
    pass.colors[0].texture = mTarget;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::DontCare;
    pass.colorStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);
    mDriver->bindPipeline(mShared->mRenderPipeline);
    mDriver->bindUniformBuffer(1, mStateBuffer, 0, static_cast<std::uint32_t>(sizeof(state)));
    mDriver->bindStorageBuffer(0, mElementBuffer, 0,
            static_cast<std::uint32_t>(sizeof(RenderElementData)) * mRenderElementMax);
    mDriver->bindStorageBuffer(1, mClusterRenderBuffer, 0, mClusterRenderBufferSize);

    const ClusterBuilderShared::Mesh* bound = nullptr;
    for (std::uint32_t i = 0; i < mRenderElementCount; ++i)
    {
        const RenderElementData& e = mRenderElements[i];
        const ClusterBuilderShared::Mesh* mesh = &mShared->mBox;
        switch (e.type)
        {
            case kElementOmniLight:
                mesh = &mShared->mSphere;
                break;
            case kElementSpotLight:
                mesh = e.hasWideSpotAngle ? &mShared->mSphere : &mShared->mCone;
                break;
            case kElementAreaLight:
            case kElementDecal:
            case kElementReflectionProbe:
                mesh = &mShared->mBox;
                break;
        }
        if (mesh != bound)
        {
            mDriver->bindVertexBuffer(0, mesh->vertexBuffer, 0);
            mDriver->bindIndexBuffer(mesh->indexBuffer);
            bound = mesh;
        }
        mDriver->bindUniformBuffer(0, mDrawParamsBuffer, i * mUniformStride,
                static_cast<std::uint32_t>(sizeof(DrawParams)));
        mDriver->drawIndexed(mesh->indexCount, 0, 1);
    }
    mDriver->endRenderPass();

    mDriver->beginComputePass();
    mDriver->bindPipeline(mShared->mStorePipeline);
    mDriver->bindUniformBuffer(0, mStoreParamsBuffer, 0,
            static_cast<std::uint32_t>(sizeof(StoreParams)));
    mDriver->bindStorageBuffer(0, mClusterRenderBuffer, 0, mClusterRenderBufferSize);
    mDriver->bindStorageBuffer(1, mClusterBuffer, 0, mClusterBufferSize);
    mDriver->bindStorageBuffer(2, mElementBuffer, 0,
            static_cast<std::uint32_t>(sizeof(RenderElementData)) * mRenderElementMax);
    mDriver->dispatch((mClusterWidth + 7) / 8, (mClusterHeight + 7) / 8, 1);
    mDriver->endComputePass();
    return true;
}

prisma::BufferHandle ClusterBuilder::getClusterBuffer() const
{
    return mClusterBuffer;
}

std::uint32_t ClusterBuilder::getClusterBufferSize() const
{
    return mClusterBufferSize;
}

std::uint32_t ClusterBuilder::getClusterSize() const
{
    return mClusterSize;
}

std::uint32_t ClusterBuilder::getClusterShift() const
{
    return shiftFromPowerOf2(mClusterSize);
}

std::uint32_t ClusterBuilder::getClusterWidth() const
{
    return mClusterWidth;
}

std::uint32_t ClusterBuilder::getClusterHeight() const
{
    return mClusterHeight;
}

std::uint32_t ClusterBuilder::getClusterTypeSize() const
{
    return mClusterWidth * mClusterHeight * (mMaxElementsByType / 32 + 32);
}

std::uint32_t ClusterBuilder::getMaxClusterElements() const
{
    return mMaxElementsByType;
}

std::uint32_t ClusterBuilder::getMaxClusterElementCountDiv32() const
{
    return mMaxElementsByType / 32;
}

std::uint32_t ClusterBuilder::getScreenWidth() const
{
    return mScreenWidth;
}

std::uint32_t ClusterBuilder::getScreenHeight() const
{
    return mScreenHeight;
}

std::uint32_t ClusterBuilder::getClusterCountByType(ElementType type) const
{
    return type < kElementTypeMax ? mClusterCountByType[type] : 0;
}

std::uint32_t ClusterBuilder::getElementCount() const
{
    return mRenderElementCount;
}

float ClusterBuilder::getZNear() const
{
    return mZNear;
}

float ClusterBuilder::getZFar() const
{
    return mZFar;
}

} // namespace prisma3d
