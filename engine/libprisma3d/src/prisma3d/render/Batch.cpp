#include "prisma3d/render/Batch.h"
#include "prisma3d/render/ShaderVariants.h"

#include "BatchFont.h"

#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{


static const int FONT_COLS = 16;
static const int FONT_ATLAS_W = 128; // 16 cols * 8px
static const int FONT_ATLAS_H = 48;  // 6 rows * 8px

static const float PI = 3.14159265359f;
static const float DEG2RAD = PI / 180.0f;

namespace
{

void applyBlend(prisma::PipelineDesc& state, BatchRenderer::BlendMode mode)
{
    state.blend = true;
    state.srcAlpha = prisma::BlendFactor::One;
    state.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
    switch (mode)
    {
        case BatchRenderer::BlendMode::Alpha:
            state.srcColor = prisma::BlendFactor::SrcAlpha;
            state.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
            break;
        case BatchRenderer::BlendMode::Additive:
            state.srcColor = prisma::BlendFactor::SrcAlpha;
            state.dstColor = prisma::BlendFactor::One;
            break;
        case BatchRenderer::BlendMode::Multiplied:
            state.srcColor = prisma::BlendFactor::DstColor;
            state.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
            break;
        case BatchRenderer::BlendMode::AddColors:
            state.srcColor = prisma::BlendFactor::One;
            state.dstColor = prisma::BlendFactor::One;
            break;
        case BatchRenderer::BlendMode::SubtractColors:
            state.srcColor = prisma::BlendFactor::One;
            state.dstColor = prisma::BlendFactor::One;
            state.colorBlendOp = prisma::BlendOp::ReverseSubtract;
            break;
    }
}

std::uint32_t alignValue(std::uint32_t value, std::uint32_t alignment)
{
    return alignment > 1 ? (value + alignment - 1) / alignment * alignment : value;
}

const std::uint32_t kUniformBytes = 80;

} // namespace

BatchRenderer::BatchRenderer()
        : mCurrentMatrix(Math::Mat4::Identity()),
          mCurrentColor(0xFFFFFFFF),
          mCurrentMode(ModeTriangles),
          mInBeginEnd(false),
          mCurrentBlendMode(BlendMode::Alpha),
          mSpace(Space::Screen),
          mDepthTestEnabled(false),
          mDepthWriteEnabled(false),
          mBlendEnabled(true),
          mCullFaceEnabled(false),
          mClipEnabled(false),
          mDriver(nullptr),
          mVertexCapacity(0),
          mIndexCapacity(0),
          mUniformStride(0)
{
    mCurrentTexcoord[0] = 0.0f;
    mCurrentTexcoord[1] = 0.0f;
}

BatchRenderer::~BatchRenderer()
{
    shutdown();
}

bool BatchRenderer::init(prisma::Driver* driver)
{
    return init(driver, Config());
}

bool BatchRenderer::init(prisma::Driver* driver, const Config& config)
{
    if (!driver) return false;
    shutdown();
    mDriver = driver;
    mConfig = config;
    mVertices.reserve(mConfig.maxVertices);
    mIndices.reserve(mConfig.maxVertices);
    mMatrixStack.reserve(mConfig.stackDepth);

    const prisma::Caps& caps = driver->caps();
    mVertexShader = driver->createShader(prisma3d::shaderDesc(batch_vert, caps));
    mFragmentShader = driver->createShader(prisma3d::shaderDesc(batch_frag, caps));

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Nearest;
    samplerDesc.magFilter = prisma::Filter::Nearest;
    samplerDesc.mipFilter = prisma::MipFilter::None;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "batch";
    mSampler = driver->createSampler(samplerDesc);

    const unsigned char whitePixel[4] = { 255, 255, 255, 255 };
    prisma::TextureDesc whiteDesc;
    whiteDesc.format = prisma::TextureFormat::RGBA8;
    whiteDesc.width = 1;
    whiteDesc.height = 1;
    whiteDesc.data = whitePixel;
    whiteDesc.debugName = "batch white";
    mWhiteTexture = driver->createTexture(whiteDesc);

    ct::Vector<unsigned char> atlas;
    atlas.resize(static_cast<std::size_t>(FONT_ATLAS_W) * FONT_ATLAS_H * 4);
    memset(atlas.data(), 0, atlas.size());
    for (int g = 0; g < 96; ++g)
    {
        const int cellX = (g % FONT_COLS) * 8;
        const int cellY = (g / FONT_COLS) * 8;
        for (int row = 0; row < 8; ++row)
        {
            const unsigned char bits = kFont8x8[g][row];
            for (int col = 0; col < 8; ++col)
            {
                if (!((bits >> col) & 1)) continue;
                unsigned char* p = &atlas[((cellY + row) * FONT_ATLAS_W + cellX + col) * 4];
                p[0] = p[1] = p[2] = p[3] = 255;
            }
        }
    }
    prisma::TextureDesc fontDesc;
    fontDesc.format = prisma::TextureFormat::RGBA8;
    fontDesc.width = FONT_ATLAS_W;
    fontDesc.height = FONT_ATLAS_H;
    fontDesc.data = atlas.data();
    fontDesc.debugName = "batch font";
    mFontTexture = driver->createTexture(fontDesc);

    mUniformStride = alignValue(kUniformBytes, caps.uniformBufferOffsetAlignment);
    prisma::BufferDesc uniformDesc;
    uniformDesc.usage = prisma::BufferUsage::Uniform;
    uniformDesc.update = prisma::BufferUpdate::Dynamic;
    uniformDesc.size = mUniformStride * static_cast<std::uint32_t>(mConfig.uniformSlots);
    uniformDesc.debugName = "batch uniforms";
    mUniformBuffer = driver->createBuffer(uniformDesc);

    const bool ready = mVertexShader.valid() && mFragmentShader.valid() && mSampler.valid() &&
                       mWhiteTexture.valid() && mFontTexture.valid() && mUniformBuffer.valid();
    if (!ready)
    {
        log_error("render: could not create the batch resources");
        shutdown();
    }
    return ready;
}

void BatchRenderer::shutdown()
{
    if (!mDriver) return;
    for (std::size_t i = 0; i < mPipelines.size(); ++i)
        if (mPipelines[i].pipeline.valid()) mDriver->destroy(mPipelines[i].pipeline);
    mPipelines.clear();
    if (mVertexShader.valid()) mDriver->destroy(mVertexShader);
    if (mFragmentShader.valid()) mDriver->destroy(mFragmentShader);
    if (mVertexBuffer.valid()) mDriver->destroy(mVertexBuffer);
    if (mIndexBuffer.valid()) mDriver->destroy(mIndexBuffer);
    if (mUniformBuffer.valid()) mDriver->destroy(mUniformBuffer);
    if (mWhiteTexture.valid()) mDriver->destroy(mWhiteTexture);
    if (mFontTexture.valid()) mDriver->destroy(mFontTexture);
    if (mSampler.valid()) mDriver->destroy(mSampler);
    mVertexShader = prisma::ShaderHandle();
    mFragmentShader = prisma::ShaderHandle();
    mVertexBuffer = prisma::BufferHandle();
    mIndexBuffer = prisma::BufferHandle();
    mUniformBuffer = prisma::BufferHandle();
    mWhiteTexture = prisma::TextureHandle();
    mFontTexture = prisma::TextureHandle();
    mSampler = prisma::SamplerHandle();
    mCurrentTexture = prisma::TextureHandle();
    mVertexCapacity = 0;
    mIndexCapacity = 0;
    mDriver = nullptr;
    mVertices.clear();
    mIndices.clear();
    mDrawCalls.clear();
    mMatrixStack.clear();
}

void BatchRenderer::setSpace(Space space)
{
    mSpace = space;
}

BatchRenderer::Space BatchRenderer::space() const
{
    return mSpace;
}

void BatchRenderer::resetStats()
{
    mStats = Stats();
}

Math::Mat4 BatchRenderer::screenProjection(float width, float height)
{
    return Math::Mat4::Ortho(0.0f, width, height, 0.0f, -1.0f, 1.0f);
}

bool BatchRenderer::empty(Space space) const
{
    for (std::size_t i = 0; i < mDrawCalls.size(); ++i)
        if (mDrawCalls[i].space == space && mDrawCalls[i].vertexCount > 0) return false;
    return true;
}

void BatchRenderer::clear()
{
    if (mInBeginEnd) end();
    mVertices.clear();
    mIndices.clear();
    mDrawCalls.clear();
}

void BatchRenderer::begin(int mode)
{
    if (mInBeginEnd)
    {
        log_error("render: BatchRenderer::begin() called without end()");
        end();
    }

    mInBeginEnd = true;
    mCurrentMode = mode;

    if (mDrawCalls.empty() || mDrawCalls.back().mode != mode ||
            mDrawCalls.back().texture != mCurrentTexture ||
            mDrawCalls.back().blendMode != mCurrentBlendMode ||
            mDrawCalls.back().depthTest != mDepthTestEnabled ||
            mDrawCalls.back().depthWrite != mDepthWriteEnabled ||
            mDrawCalls.back().blend != mBlendEnabled ||
            mDrawCalls.back().cullFace != mCullFaceEnabled || mDrawCalls.back().space != mSpace)
    {
        DrawCall call;
        call.mode = mode;
        call.vertexCount = 0;
        call.vertexAlignment = mVertices.size();
        call.firstIndex = 0;
        call.texture = mCurrentTexture;
        call.blendMode = mCurrentBlendMode;
        call.depthTest = mDepthTestEnabled;
        call.depthWrite = mDepthWriteEnabled;
        call.blend = mBlendEnabled;
        call.cullFace = mCullFaceEnabled;
        call.space = mSpace;
        mDrawCalls.push_back(call);
    }
}

void BatchRenderer::vertex3(float x, float y, float z)
{
    if (!mInBeginEnd) return;

    applyTransform(x, y, z);

    Vertex v;
    v.x = x;
    v.y = y;
    v.z = z;
    v.u = mCurrentTexcoord[0];
    v.v = mCurrentTexcoord[1];
    unpackColor(mCurrentColor, v.r, v.g, v.b, v.a);
    mVertices.push_back(v);
}

bool BatchRenderer::ensureBuffer(prisma::BufferHandle& buffer, std::uint32_t& capacity,
        std::uint32_t size, prisma::BufferUsage usage, prisma::IndexFormat format, const char* name)
{
    if (buffer.valid() && capacity >= size) return true;
    std::uint32_t grown = capacity * 2;
    if (grown < size) grown = size;
    if (grown < 4096) grown = 4096;
    if (buffer.valid()) mDriver->destroy(buffer);
    buffer = prisma::BufferHandle();
    capacity = 0;
    prisma::BufferDesc desc;
    desc.usage = usage;
    desc.update = prisma::BufferUpdate::Dynamic;
    desc.size = grown;
    desc.indexFormat = format;
    desc.debugName = name;
    buffer = mDriver->createBuffer(desc);
    if (!buffer.valid())
    {
        log_error("render: could not create the batch buffer");
        return false;
    }
    capacity = grown;
    return true;
}

void BatchRenderer::upload()
{
    if (!mDriver || mVertices.empty() || mDrawCalls.empty()) return;
    if (mInBeginEnd) end();

    const std::uint32_t vertexBytes = static_cast<std::uint32_t>(mVertices.size() * sizeof(Vertex));
    if (!ensureBuffer(mVertexBuffer, mVertexCapacity, vertexBytes, prisma::BufferUsage::Vertex,
                prisma::IndexFormat::UInt16, "batch vertices"))
        return;
    mDriver->updateBuffer(mVertexBuffer, 0, mVertices.data(), vertexBytes);

    mIndices.clear();
    for (std::size_t i = 0; i < mDrawCalls.size(); ++i)
    {
        DrawCall& call = mDrawCalls[i];
        call.firstIndex = static_cast<std::uint32_t>(mIndices.size());
        if (call.mode != ModeQuads) continue;
        const std::size_t quadCount = call.vertexCount / 4;
        for (std::size_t q = 0; q < quadCount; ++q)
        {
            const std::uint32_t base = static_cast<std::uint32_t>(call.vertexAlignment + q * 4);
            mIndices.push_back(base + 0);
            mIndices.push_back(base + 1);
            mIndices.push_back(base + 2);
            mIndices.push_back(base + 0);
            mIndices.push_back(base + 2);
            mIndices.push_back(base + 3);
        }
    }
    if (!mIndices.empty())
    {
        const std::uint32_t indexBytes =
                static_cast<std::uint32_t>(mIndices.size() * sizeof(std::uint32_t));
        if (ensureBuffer(mIndexBuffer, mIndexCapacity, indexBytes, prisma::BufferUsage::Index,
                    prisma::IndexFormat::UInt32, "batch indices"))
            mDriver->updateBuffer(mIndexBuffer, 0, mIndices.data(), indexBytes);
    }
}

void BatchRenderer::setUniforms(std::uint32_t slot, const Math::Mat4& projection, bool linearColors)
{
    if (!mDriver || !mUniformBuffer.valid()) return;
    if (slot >= mConfig.uniformSlots) slot = static_cast<std::uint32_t>(mConfig.uniformSlots) - 1;
    float data[20];
    memcpy(data, &projection, sizeof(float) * 16);
    data[16] = linearColors ? 1.0f : 0.0f;
    data[17] = data[18] = data[19] = 0.0f;
    mDriver->updateBuffer(mUniformBuffer, slot * mUniformStride, data, sizeof(data));
}

prisma::PipelineHandle BatchRenderer::pipelineFor(const DrawCall& call, std::uint32_t samples,
        prisma::TextureFormat screenTarget)
{
    const bool world = call.space == Space::World;
    const std::uint32_t key =
            static_cast<std::uint32_t>(call.mode) | (call.depthTest && world ? 1u << 8 : 0u) |
            (call.depthWrite && world ? 1u << 9 : 0u) | (call.blend ? 1u << 10 : 0u) |
            (call.cullFace ? 1u << 11 : 0u) | (static_cast<std::uint32_t>(call.blendMode) << 12) |
            (world ? 1u << 16 : 0u) | ((world && samples > 1 ? samples : 1u) << 17) |
            (!world && screenTarget != prisma::TextureFormat::None
                            ? (static_cast<std::uint32_t>(screenTarget) & 0xffu) << 24
                            : 0u);
    for (std::size_t i = 0; i < mPipelines.size(); ++i)
        if (mPipelines[i].key == key) return mPipelines[i].pipeline;

    prisma::PipelineDesc desc;
    desc.vertexShader = mVertexShader;
    desc.fragmentShader = mFragmentShader;
    desc.vertexBuffers[0].stride = sizeof(Vertex);
    desc.vertexBufferCount = 1;
    desc.attributes[0].location = 0;
    desc.attributes[0].format = prisma::VertexFormat::Float3;
    desc.attributes[0].offset = 0;
    desc.attributes[1].location = 1;
    desc.attributes[1].format = prisma::VertexFormat::Float2;
    desc.attributes[1].offset = offsetof(Vertex, u);
    desc.attributes[2].location = 2;
    desc.attributes[2].format = prisma::VertexFormat::UByte4Norm;
    desc.attributes[2].offset = offsetof(Vertex, r);
    desc.attributeCount = 3;
    desc.topology = call.mode == ModeLines ? prisma::Topology::Lines : prisma::Topology::Triangles;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "BatchUniforms";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "u_texture";
    desc.textures[0].slot = 0;
    if (call.blend) applyBlend(desc, call.blendMode);
    if (world)
    {
        desc.targets.window = false;
        desc.targets.samples = samples;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::Depth32F;
        desc.depthTest = call.depthTest;
        desc.depthWrite = call.depthWrite;
        desc.depthCompare = prisma::CompareOp::LessEqual;
    }
    else
    {
        desc.depthTest = false;
        desc.depthWrite = false;
        if (screenTarget != prisma::TextureFormat::None)
        {
            desc.targets.window = false;
            desc.targets.colors[0] = screenTarget;
            desc.targets.colorCount = 1;
            desc.targets.depth = prisma::TextureFormat::None;
        }
    }
    desc.cullMode = call.cullFace ? prisma::CullMode::Back : prisma::CullMode::None;
    desc.frontFace = prisma::FrontFace::CounterClockwise;
    desc.debugName = "batch";

    CachedPipeline cached;
    cached.key = key;
    cached.pipeline = mDriver->createPipeline(desc);
    if (!cached.pipeline.valid()) log_error("render: could not create a batch pipeline");
    mPipelines.push_back(cached);
    return cached.pipeline;
}

void BatchRenderer::draw(Space space, std::uint32_t slot, std::uint32_t samples,
        prisma::TextureFormat screenTarget)
{
    if (!mDriver || !mVertexBuffer.valid()) return;
    if (slot >= mConfig.uniformSlots) slot = static_cast<std::uint32_t>(mConfig.uniformSlots) - 1;
    prisma::PipelineHandle bound;
    for (std::size_t i = 0; i < mDrawCalls.size(); ++i)
    {
        const DrawCall& call = mDrawCalls[i];
        if (call.space != space || call.vertexCount == 0) continue;
        const prisma::PipelineHandle pipeline = pipelineFor(call, samples, screenTarget);
        if (!pipeline.valid()) continue;
        if (!bound.valid() || bound != pipeline)
        {
            mDriver->bindPipeline(pipeline);
            bound = pipeline;
        }
        mDriver->bindUniformBuffer(0, mUniformBuffer, slot * mUniformStride, kUniformBytes);
        mDriver->bindTexture(0, call.texture.valid() ? call.texture : mWhiteTexture, mSampler);
        mDriver->bindVertexBuffer(0, mVertexBuffer, 0);
        if (call.mode == ModeQuads)
        {
            if (!mIndexBuffer.valid()) continue;
            mDriver->bindIndexBuffer(mIndexBuffer);
            mDriver->drawIndexed(static_cast<std::uint32_t>(call.vertexCount / 4 * 6),
                    call.firstIndex);
            mStats.indicesDrawn += call.vertexCount / 4 * 6;
        }
        else
            mDriver->draw(static_cast<std::uint32_t>(call.vertexCount),
                    static_cast<std::uint32_t>(call.vertexAlignment));
        ++mStats.drawCalls;
        mStats.verticesDrawn += call.vertexCount;
    }
}


void BatchRenderer::pushMatrix()
{
    if (mMatrixStack.size() < mConfig.stackDepth)
    {
        mMatrixStack.push_back(mCurrentMatrix);
    }
}

void BatchRenderer::popMatrix()
{
    if (!mMatrixStack.empty())
    {
        mCurrentMatrix = mMatrixStack.back();
        mMatrixStack.pop_back();
    }
    else
    {
        mCurrentMatrix = Math::Mat4::Identity();
    }
}

void BatchRenderer::loadIdentity()
{
    mCurrentMatrix = Math::Mat4::Identity();
}

void BatchRenderer::translate(float x, float y, float z)
{
    mCurrentMatrix = mCurrentMatrix * Math::Mat4::Translation(Math::Vec3(x, y, z));
}

void BatchRenderer::rotate(float angleDeg, float axisX, float axisY, float axisZ)
{
    Math::Vec3 axis(axisX, axisY, axisZ);
    if (axis.LengthSquared() <= 0.0f) return;
    const float angleRad = angleDeg * (PI / 180.0f);
    mCurrentMatrix =
            mCurrentMatrix * Math::Quaternion::FromAxisAngle(axis.Normalized(), angleRad).ToMat4();
}

void BatchRenderer::scale(float x, float y, float z)
{
    mCurrentMatrix = mCurrentMatrix * Math::Mat4::Scale(Math::Vec3(x, y, z));
}

void BatchRenderer::setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
    mCurrentColor = packColor(r, g, b, a);
}

void BatchRenderer::setColor(float r, float g, float b, float a)
{
    mCurrentColor = packColor(static_cast<unsigned char>(r * 255.0f + 0.5f),
            static_cast<unsigned char>(g * 255.0f + 0.5f),
            static_cast<unsigned char>(b * 255.0f + 0.5f),
            static_cast<unsigned char>(a * 255.0f + 0.5f));
}

void BatchRenderer::setTexture(prisma::TextureHandle texture)
{
    if (mCurrentTexture != texture)
    {
        mCurrentTexture = texture;
        if (true)
        {
            mStats.textureSwitches++;
        }
    }
}

void BatchRenderer::setBlendMode(BlendMode mode)
{
    mCurrentBlendMode = mode;
}

void BatchRenderer::setTexcoord(float u, float v)
{
    mCurrentTexcoord[0] = u;
    mCurrentTexcoord[1] = v;
}

void BatchRenderer::setDefault3DState()
{
    mDepthTestEnabled = true;
    mDepthWriteEnabled = true;
    mBlendEnabled = false;
    mCullFaceEnabled = false;
}

void BatchRenderer::setClipRect(float x, float y, float width, float height)
{
    mClipRect = FloatRect{ x, y, width, height };
    mClipEnabled = true;
}

void BatchRenderer::setClipRect(const FloatRect& rect)
{
    mClipRect = rect;
    mClipEnabled = true;
}

void BatchRenderer::clearClipRect()
{
    mClipEnabled = false;
}

// Sutherland-Hodgman polygon clip; `out` needs inCount+4 vertices.
int BatchRenderer::clipPolygonToRect(const ClipVertex* in, int inCount, ClipVertex* out) const
{
    const float minX = mClipRect.x;
    const float minY = mClipRect.y;
    const float maxX = mClipRect.x + mClipRect.width;
    const float maxY = mClipRect.y + mClipRect.height;

    ClipVertex tmp[16];
    const ClipVertex* src = in;
    int srcCount = inCount;

    for (int plane = 0; plane < 4; ++plane)
    {
        ClipVertex* dst = (plane % 2 == 0) ? tmp : out;
        int dstCount = 0;

        for (int i = 0; i < srcCount; ++i)
        {
            const ClipVertex& cur = src[i];
            const ClipVertex& prev = src[(i + srcCount - 1) % srcCount];

            bool curIn = false, prevIn = false;
            switch (plane)
            {
                case 0:
                    curIn = cur.x >= minX;
                    prevIn = prev.x >= minX;
                    break;
                case 1:
                    curIn = cur.x <= maxX;
                    prevIn = prev.x <= maxX;
                    break;
                case 2:
                    curIn = cur.y >= minY;
                    prevIn = prev.y >= minY;
                    break;
                case 3:
                    curIn = cur.y <= maxY;
                    prevIn = prev.y <= maxY;
                    break;
            }

            if (curIn != prevIn)
            {
                float t = 0.f;
                switch (plane)
                {
                    case 0:
                        t = (minX - prev.x) / (cur.x - prev.x);
                        break;
                    case 1:
                        t = (maxX - prev.x) / (cur.x - prev.x);
                        break;
                    case 2:
                        t = (minY - prev.y) / (cur.y - prev.y);
                        break;
                    case 3:
                        t = (maxY - prev.y) / (cur.y - prev.y);
                        break;
                }
                ClipVertex isect;
                isect.x = prev.x + t * (cur.x - prev.x);
                isect.y = prev.y + t * (cur.y - prev.y);
                isect.u = prev.u + t * (cur.u - prev.u);
                isect.v = prev.v + t * (cur.v - prev.v);
                if (dstCount < 16) dst[dstCount++] = isect;
            }
            if (curIn)
            {
                if (dstCount < 16) dst[dstCount++] = cur;
            }
        }

        src = dst;
        srcCount = dstCount;
        if (srcCount == 0) return 0;
    }

    if (src == tmp)
    {
        for (int i = 0; i < srcCount; ++i) out[i] = tmp[i];
    }
    return srcCount;
}

// Liang-Barsky line clip; false if fully outside.
bool BatchRenderer::clipSegmentToRect(float& x0, float& y0, float& x1, float& y1) const
{
    const float minX = mClipRect.x;
    const float minY = mClipRect.y;
    const float maxX = mClipRect.x + mClipRect.width;
    const float maxY = mClipRect.y + mClipRect.height;

    float dx = x1 - x0;
    float dy = y1 - y0;
    float tMin = 0.f, tMax = 1.f;

    float p[4] = { -dx, dx, -dy, dy };
    float q[4] = { x0 - minX, maxX - x0, y0 - minY, maxY - y0 };

    for (int i = 0; i < 4; ++i)
    {
        if (p[i] == 0.f)
        {
            if (q[i] < 0.f) return false;
        }
        else
        {
            float t = q[i] / p[i];
            if (p[i] < 0.f) tMin = fmaxf(tMin, t);
            else
                tMax = fminf(tMax, t);
        }
    }

    if (tMin > tMax) return false;

    float nx0 = x0 + tMin * dx, ny0 = y0 + tMin * dy;
    float nx1 = x0 + tMax * dx, ny1 = y0 + tMax * dy;
    x0 = nx0;
    y0 = ny0;
    x1 = nx1;
    y1 = ny1;
    return true;
}

void BatchRenderer::submitClippedQuad(float x0, float y0, float x1, float y1, float x2, float y2,
        float x3, float y3)
{
    ClipVertex in[4] = {
        { x0, y0, 0.f, 0.f },
        { x1, y1, 1.f, 0.f },
        { x2, y2, 1.f, 1.f },
        { x3, y3, 0.f, 1.f },
    };
    ClipVertex out[16];
    int n = clipPolygonToRect(in, 4, out);
    if (n < 3) return;

    for (int i = 1; i + 1 < n; ++i)
    {
        submitVertex(out[0].x, out[0].y, 0.f, out[0].u, out[0].v);
        submitVertex(out[i].x, out[i].y, 0.f, out[i].u, out[i].v);
        submitVertex(out[i + 1].x, out[i + 1].y, 0.f, out[i + 1].u, out[i + 1].v);
    }
}

void BatchRenderer::submitClippedLine(float x0, float y0, float x1, float y1)
{
    if (!clipSegmentToRect(x0, y0, x1, y1)) return;
    submitVertex(x0, y0, 0.f);
    submitVertex(x1, y1, 0.f);
}

void BatchRenderer::emitLine(float x0, float y0, float x1, float y1)
{
    if (mClipEnabled)
    {
        float z0 = 0.f, z1 = 0.f;
        applyTransform(x0, y0, z0);
        applyTransform(x1, y1, z1);
        submitClippedLine(x0, y0, x1, y1);
    }
    else
    {
        vertex2(x0, y0);
        vertex2(x1, y1);
    }
}

void BatchRenderer::emitTriangle(float x0, float y0, float x1, float y1, float x2, float y2)
{
    emitTexturedTriangle(x0, y0, 0.f, 0.f, x1, y1, 0.f, 0.f, x2, y2, 0.f, 0.f);
}

void BatchRenderer::emitTexturedTriangle(float x0, float y0, float u0, float v0, float x1, float y1,
        float u1, float v1, float x2, float y2, float u2, float v2)
{
    if (mClipEnabled)
    {
        float z0 = 0.f, z1 = 0.f, z2 = 0.f;
        applyTransform(x0, y0, z0);
        applyTransform(x1, y1, z1);
        applyTransform(x2, y2, z2);

        ClipVertex in[3] = { { x0, y0, u0, v0 }, { x1, y1, u1, v1 }, { x2, y2, u2, v2 } };
        ClipVertex out[16];
        int n = clipPolygonToRect(in, 3, out);
        for (int i = 1; i + 1 < n; ++i)
        {
            submitVertex(out[0].x, out[0].y, 0.f, out[0].u, out[0].v);
            submitVertex(out[i].x, out[i].y, 0.f, out[i].u, out[i].v);
            submitVertex(out[i + 1].x, out[i + 1].y, 0.f, out[i + 1].u, out[i + 1].v);
        }
    }
    else
    {
        setTexcoord(u0, v0);
        vertex2(x0, y0);
        setTexcoord(u1, v1);
        vertex2(x1, y1);
        setTexcoord(u2, v2);
        vertex2(x2, y2);
    }
}

void BatchRenderer::vertex2(float x, float y)
{
    vertex3(x, y, 0.0f);
}


void BatchRenderer::end()
{
    if (!mInBeginEnd) return;

    mInBeginEnd = false;

    if (!mDrawCalls.empty())
    {
        mDrawCalls.back().vertexCount = mVertices.size() - mDrawCalls.back().vertexAlignment;
    }
}

void BatchRenderer::drawLine(float x0, float y0, float x1, float y1)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    if (mClipEnabled)
    {
        float z = 0.f;
        applyTransform(x0, y0, z);
        z = 0.f;
        applyTransform(x1, y1, z);
        submitClippedLine(x0, y0, x1, y1);
    }
    else
    {
        vertex2(x0, y0);
        vertex2(x1, y1);
    }
    end();
}

void BatchRenderer::drawTriangle(float x1, float y1, float x2, float y2, float x3, float y3)
{
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    emitTriangle(x1, y1, x2, y2, x3, y3);
    end();
}

void BatchRenderer::drawRect(float x, float y, float width, float height, bool fill)
{
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        if (mClipEnabled)
        {
            float x0 = x, y0 = y, z0 = 0.f;
            float x1 = x + width, y1 = y, z1 = 0.f;
            float x2 = x + width, y2 = y + height, z2 = 0.f;
            float x3 = x, y3 = y + height, z3 = 0.f;
            applyTransform(x0, y0, z0);
            applyTransform(x1, y1, z1);
            applyTransform(x2, y2, z2);
            applyTransform(x3, y3, z3);
            submitClippedQuad(x0, y0, x1, y1, x2, y2, x3, y3);
        }
        else
        {
            vertex2(x, y);
            vertex2(x + width, y);
            vertex2(x, y + height);
            vertex2(x + width, y);
            vertex2(x + width, y + height);
            vertex2(x, y + height);
        }
        end();
    }
    else
    {
        begin(ModeLines);
        if (mClipEnabled)
        {
            float corners[4][2] = { { x, y }, { x + width, y }, { x + width, y + height },
                { x, y + height } };
            for (int i = 0; i < 4; ++i)
            {
                float x0 = corners[i][0], y0 = corners[i][1], z0 = 0.f;
                float x1 = corners[(i + 1) % 4][0], y1 = corners[(i + 1) % 4][1], z1 = 0.f;
                applyTransform(x0, y0, z0);
                applyTransform(x1, y1, z1);
                submitClippedLine(x0, y0, x1, y1);
            }
        }
        else
        {
            vertex2(x, y);
            vertex2(x + width, y);
            vertex2(x + width, y);
            vertex2(x + width, y + height);
            vertex2(x + width, y + height);
            vertex2(x, y + height);
            vertex2(x, y + height);
            vertex2(x, y);
        }
        end();
    }
}

void BatchRenderer::drawQuad(float x, float y, float width, float height)
{
    setTexture(mWhiteTexture);
    begin(ModeQuads);
    vertex2(x, y);
    vertex2(x + width, y);
    vertex2(x + width, y + height);
    vertex2(x, y + height);
    end();
}

void BatchRenderer::drawCircle(float cx, float cy, float radius, int segments)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float angle1 = (float) i * 2.0f * PI / (float) segments;
        float angle2 = (float) (i + 1) * 2.0f * PI / (float) segments;

        float x1 = cx + cosf(angle1) * radius;
        float y1 = cy + sinf(angle1) * radius;
        float x2 = cx + cosf(angle2) * radius;
        float y2 = cy + sinf(angle2) * radius;

        emitLine(x1, y1, x2, y2);
    }
    end();
}

void BatchRenderer::drawPolyline(const float* xyPairs, int pointCount)
{
    if (!xyPairs || pointCount < 2) return;

    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < pointCount - 1; ++i)
    {
        emitLine(xyPairs[i * 2], xyPairs[i * 2 + 1], xyPairs[(i + 1) * 2],
                xyPairs[(i + 1) * 2 + 1]);
    }
    end();
}

void BatchRenderer::drawThickLine(float x0, float y0, float x1, float y1, float thickness)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    if (len <= 0.0f) return;
    float nx = -dy / len * thickness * 0.5f;
    float ny = dx / len * thickness * 0.5f;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    emitTriangle(x0 + nx, y0 + ny, x0 - nx, y0 - ny, x1 - nx, y1 - ny);
    emitTriangle(x0 + nx, y0 + ny, x1 - nx, y1 - ny, x1 + nx, y1 + ny);
    end();
}

void BatchRenderer::drawEllipse(float cx, float cy, float rx, float ry, bool fill, int segments)
{
    if (segments < 3) segments = 3;
    float step = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            emitTriangle(cx, cy, cx + rx * cosf(a0), cy + ry * sinf(a0), cx + rx * cosf(a1),
                    cy + ry * sinf(a1));
        }
        end();
    }
    else
    {
        begin(ModeLines);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            emitLine(cx + rx * cosf(a0), cy + ry * sinf(a0), cx + rx * cosf(a1),
                    cy + ry * sinf(a1));
        }
        end();
    }
}

void BatchRenderer::drawArc(float cx, float cy, float radius, float startDeg, float endDeg,
        int segments)
{
    if (segments < 1) segments = 1;
    float a0 = startDeg * DEG2RAD;
    float step = (endDeg - startDeg) * DEG2RAD / (float) segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float t0 = a0 + i * step, t1 = a0 + (i + 1) * step;
        emitLine(cx + radius * cosf(t0), cy + radius * sinf(t0), cx + radius * cosf(t1),
                cy + radius * sinf(t1));
    }
    end();
}

void BatchRenderer::drawRing(float cx, float cy, float rInner, float rOuter, bool fill,
        int segments)
{
    if (segments < 3) segments = 3;
    float step = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            float ix0 = cx + rInner * cosf(a0), iy0 = cy + rInner * sinf(a0);
            float ox0 = cx + rOuter * cosf(a0), oy0 = cy + rOuter * sinf(a0);
            float ix1 = cx + rInner * cosf(a1), iy1 = cy + rInner * sinf(a1);
            float ox1 = cx + rOuter * cosf(a1), oy1 = cy + rOuter * sinf(a1);
            emitTriangle(ix0, iy0, ox0, oy0, ox1, oy1);
            emitTriangle(ix0, iy0, ox1, oy1, ix1, iy1);
        }
        end();
    }
    else
    {
        drawCircle(cx, cy, rInner, segments);
        drawCircle(cx, cy, rOuter, segments);
    }
}

void BatchRenderer::drawPolygon(float cx, float cy, int sides, float radius, float rotDeg,
        bool fill)
{
    if (sides < 3) sides = 3;
    float step = 2.0f * PI / (float) sides;
    float rot = rotDeg * DEG2RAD;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < sides; ++i)
        {
            float a0 = rot + i * step, a1 = rot + (i + 1) * step;
            emitTriangle(cx, cy, cx + radius * cosf(a0), cy + radius * sinf(a0),
                    cx + radius * cosf(a1), cy + radius * sinf(a1));
        }
        end();
    }
    else
    {
        begin(ModeLines);
        for (int i = 0; i < sides; ++i)
        {
            float a0 = rot + i * step, a1 = rot + (i + 1) * step;
            emitLine(cx + radius * cosf(a0), cy + radius * sinf(a0), cx + radius * cosf(a1),
                    cy + radius * sinf(a1));
        }
        end();
    }
}

void BatchRenderer::drawLine3D(float x0, float y0, float z0, float x1, float y1, float z1)
{
    SpaceScope scope(*this, Space::World);
    setTexture(mWhiteTexture);
    begin(ModeLines);
    vertex3(x0, y0, z0);
    vertex3(x1, y1, z1);
    end();
}

void BatchRenderer::drawTriangle3D(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    SpaceScope scope(*this, Space::World);
    begin(ModeTriangles);
    vertex3(a.x, a.y, a.z);
    vertex3(b.x, b.y, b.z);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawTriangle3D(const Math::Vec3& a, const Math::Vec2& uvA, const Math::Vec3& b,
        const Math::Vec2& uvB, const Math::Vec3& c, const Math::Vec2& uvC)
{
    SpaceScope scope(*this, Space::World);
    begin(ModeTriangles);
    setTexcoord(uvA.x, uvA.y);
    vertex3(a.x, a.y, a.z);
    setTexcoord(uvB.x, uvB.y);
    vertex3(b.x, b.y, b.z);
    setTexcoord(uvC.x, uvC.y);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawTriangle3D(const Math::Vec3& a, const Math::Vec2& uvA, std::uint32_t colorA,
        const Math::Vec3& b, const Math::Vec2& uvB, std::uint32_t colorB, const Math::Vec3& c,
        const Math::Vec2& uvC, std::uint32_t colorC)
{
    SpaceScope scope(*this, Space::World);
    begin(ModeTriangles);
    mCurrentColor = colorA;
    setTexcoord(uvA.x, uvA.y);
    vertex3(a.x, a.y, a.z);
    mCurrentColor = colorB;
    setTexcoord(uvB.x, uvB.y);
    vertex3(b.x, b.y, b.z);
    mCurrentColor = colorC;
    setTexcoord(uvC.x, uvC.y);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawWireBox(float minX, float minY, float minZ, float maxX, float maxY,
        float maxZ)
{
    SpaceScope scope(*this, Space::World);
    setTexture(mWhiteTexture);
    begin(ModeLines);
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, minY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, maxY, minZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, maxY, maxZ);
    end();
}

void BatchRenderer::drawWireSphere(float cx, float cy, float cz, float radius, int segments)
{
    SpaceScope scope(*this, Space::World);
    float step = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        vertex3(cx + radius * cosf(a0), cy + radius * sinf(a0), cz);
        vertex3(cx + radius * cosf(a1), cy + radius * sinf(a1), cz);
        vertex3(cx + radius * cosf(a0), cy, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy, cz + radius * sinf(a1));
        vertex3(cx, cy + radius * sinf(a0), cz + radius * cosf(a0));
        vertex3(cx, cy + radius * sinf(a1), cz + radius * cosf(a1));
    }
    end();
}

void BatchRenderer::drawWireCylinder(float cx, float cy, float cz, float radius, float height,
        int segments)
{
    SpaceScope scope(*this, Space::World);
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x0, cy + halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
    }
    for (int i = 0; i < 4; ++i)
    {
        float a = i * (PI * 0.5f);
        vertex3(cx + radius * cosf(a), cy - halfH, cz + radius * sinf(a));
        vertex3(cx + radius * cosf(a), cy + halfH, cz + radius * sinf(a));
    }
    end();
}

void BatchRenderer::drawWireCapsule(float cx, float cy, float cz, float radius, float height,
        int segments)
{
    SpaceScope scope(*this, Space::World);
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float) segments;
    int hsegs = segments / 2;
    float hstep = PI / (float) hsegs;

    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        vertex3(cx + radius * cosf(a0), cy - halfH, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy - halfH, cz + radius * sinf(a1));
        vertex3(cx + radius * cosf(a0), cy + halfH, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy + halfH, cz + radius * sinf(a1));
    }
    for (int i = 0; i < 4; ++i)
    {
        float a = i * (PI * 0.5f);
        vertex3(cx + radius * cosf(a), cy - halfH, cz + radius * sinf(a));
        vertex3(cx + radius * cosf(a), cy + halfH, cz + radius * sinf(a));
    }
    for (int i = 0; i < hsegs; ++i)
    {
        float t0 = i * hstep, t1 = (i + 1) * hstep;
        vertex3(cx + radius * cosf(t0), cy + halfH + radius * sinf(t0), cz);
        vertex3(cx + radius * cosf(t1), cy + halfH + radius * sinf(t1), cz);
        vertex3(cx, cy + halfH + radius * sinf(t0), cz + radius * cosf(t0));
        vertex3(cx, cy + halfH + radius * sinf(t1), cz + radius * cosf(t1));
    }
    for (int i = 0; i < hsegs; ++i)
    {
        float t0 = i * hstep, t1 = (i + 1) * hstep;
        vertex3(cx + radius * cosf(t0), cy - halfH - radius * sinf(t0), cz);
        vertex3(cx + radius * cosf(t1), cy - halfH - radius * sinf(t1), cz);
        vertex3(cx, cy - halfH - radius * sinf(t0), cz + radius * cosf(t0));
        vertex3(cx, cy - halfH - radius * sinf(t1), cz + radius * cosf(t1));
    }
    end();
}

void BatchRenderer::drawSolidBox(float minX, float minY, float minZ, float maxX, float maxY,
        float maxZ)
{
    SpaceScope scope(*this, Space::World);
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, minY, minZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, minY, maxZ);
    vertex3(maxX, minY, maxZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, minY, maxZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(minX, maxY, maxZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, maxY, minZ);
    end();
}

void BatchRenderer::drawSolidSphere(float cx, float cy, float cz, float radius, int rings,
        int segments)
{
    SpaceScope scope(*this, Space::World);
    float rstep = PI / (float) rings;
    float sstep = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    for (int r = 0; r < rings; ++r)
    {
        float phi0 = r * rstep, phi1 = (r + 1) * rstep;
        for (int s = 0; s < segments; ++s)
        {
            float th0 = s * sstep, th1 = (s + 1) * sstep;
            float x00 = radius * sinf(phi0) * cosf(th0), y00 = radius * cosf(phi0),
                  z00 = radius * sinf(phi0) * sinf(th0);
            float x10 = radius * sinf(phi1) * cosf(th0), y10 = radius * cosf(phi1),
                  z10 = radius * sinf(phi1) * sinf(th0);
            float x01 = radius * sinf(phi0) * cosf(th1), y01 = radius * cosf(phi0),
                  z01 = radius * sinf(phi0) * sinf(th1);
            float x11 = radius * sinf(phi1) * cosf(th1), y11 = radius * cosf(phi1),
                  z11 = radius * sinf(phi1) * sinf(th1);
            vertex3(cx + x00, cy + y00, cz + z00);
            vertex3(cx + x10, cy + y10, cz + z10);
            vertex3(cx + x11, cy + y11, cz + z11);
            vertex3(cx + x00, cy + y00, cz + z00);
            vertex3(cx + x11, cy + y11, cz + z11);
            vertex3(cx + x01, cy + y01, cz + z01);
        }
    }
    end();
}

void BatchRenderer::drawSolidCylinder(float cx, float cy, float cz, float radius, float height,
        int segments)
{
    SpaceScope scope(*this, Space::World);
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float) segments;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy + halfH, cz + z0);
        vertex3(cx, cy - halfH, cz);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx, cy + halfH, cz);
        vertex3(cx + x0, cy + halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
    }
    end();
}

void BatchRenderer::drawSolidCapsule(float cx, float cy, float cz, float radius, float height,
        int rings, int segments)
{
    SpaceScope scope(*this, Space::World);
    float halfH = height * 0.5f;
    float sstep = 2.0f * PI / (float) segments;
    float hstep = (PI * 0.5f) / (float) rings;

    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * sstep, a1 = (i + 1) * sstep;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy + halfH, cz + z0);
    }
    for (int r = 0; r < rings; ++r)
    {
        float t0 = r * hstep, t1 = (r + 1) * hstep;
        for (int s = 0; s < segments; ++s)
        {
            float a0 = s * sstep, a1 = (s + 1) * sstep;
            float x00 = radius * sinf(t0) * cosf(a0), y00 = radius * cosf(t0),
                  z00 = radius * sinf(t0) * sinf(a0);
            float x10 = radius * sinf(t1) * cosf(a0), y10 = radius * cosf(t1),
                  z10 = radius * sinf(t1) * sinf(a0);
            float x01 = radius * sinf(t0) * cosf(a1), y01 = radius * cosf(t0),
                  z01 = radius * sinf(t0) * sinf(a1);
            float x11 = radius * sinf(t1) * cosf(a1), y11 = radius * cosf(t1),
                  z11 = radius * sinf(t1) * sinf(a1);
            vertex3(cx + x00, cy + halfH + y00, cz + z00);
            vertex3(cx + x10, cy + halfH + y10, cz + z10);
            vertex3(cx + x11, cy + halfH + y11, cz + z11);
            vertex3(cx + x00, cy + halfH + y00, cz + z00);
            vertex3(cx + x11, cy + halfH + y11, cz + z11);
            vertex3(cx + x01, cy + halfH + y01, cz + z01);
        }
    }
    for (int r = 0; r < rings; ++r)
    {
        float t0 = PI * 0.5f + r * hstep, t1 = PI * 0.5f + (r + 1) * hstep;
        for (int s = 0; s < segments; ++s)
        {
            float a0 = s * sstep, a1 = (s + 1) * sstep;
            float x00 = radius * sinf(t0) * cosf(a0), y00 = radius * cosf(t0),
                  z00 = radius * sinf(t0) * sinf(a0);
            float x10 = radius * sinf(t1) * cosf(a0), y10 = radius * cosf(t1),
                  z10 = radius * sinf(t1) * sinf(a0);
            float x01 = radius * sinf(t0) * cosf(a1), y01 = radius * cosf(t0),
                  z01 = radius * sinf(t0) * sinf(a1);
            float x11 = radius * sinf(t1) * cosf(a1), y11 = radius * cosf(t1),
                  z11 = radius * sinf(t1) * sinf(a1);
            vertex3(cx + x00, cy - halfH + y00, cz + z00);
            vertex3(cx + x10, cy - halfH + y10, cz + z10);
            vertex3(cx + x11, cy - halfH + y11, cz + z11);
            vertex3(cx + x00, cy - halfH + y00, cz + z00);
            vertex3(cx + x11, cy - halfH + y11, cz + z11);
            vertex3(cx + x01, cy - halfH + y01, cz + z01);
        }
    }
    end();
}

void BatchRenderer::drawAxis(float x, float y, float z, float size)
{
    SpaceScope scope(*this, Space::World);
    unsigned int savedColor = mCurrentColor;
    setTexture(mWhiteTexture);

    setColor((unsigned char) 255, (unsigned char) 0, (unsigned char) 0, (unsigned char) 255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x + size, y, z);
    end();
    setColor((unsigned char) 0, (unsigned char) 255, (unsigned char) 0, (unsigned char) 255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x, y + size, z);
    end();
    setColor((unsigned char) 0, (unsigned char) 0, (unsigned char) 255, (unsigned char) 255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x, y, z + size);
    end();

    mCurrentColor = savedColor;
}

void BatchRenderer::drawWireGrid(float y, int slices, float spacing, bool axes)
{
    SpaceScope scope(*this, Space::World);
    slices = slices < 1 ? 1 : slices;
    const float half = (float) slices * spacing * 0.5f;

    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i <= slices; ++i)
    {
        const float offset = -half + (float) i * spacing;
        const bool center = axes && fabsf(offset) < 0.0001f;

        if (center) setColor(0.8f, 0.2f, 0.2f, 1.0f);
        else
            setColor(0.45f, 0.45f, 0.45f, 1.0f);
        vertex3(offset, y, -half);
        vertex3(offset, y, half);

        if (center) setColor(0.2f, 0.8f, 0.2f, 1.0f);
        else
            setColor(0.45f, 0.45f, 0.45f, 1.0f);
        vertex3(-half, y, offset);
        vertex3(half, y, offset);
    }
    end();
    if (axes) setColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void BatchRenderer::drawTexture(prisma::TextureHandle texture, float dstX, float dstY, float dstW,
        float dstH, float srcX, float srcY, float srcW, float srcH, float pivotX, float pivotY,
        float rotationDeg)
{
    const prisma::TextureHandle previous = mCurrentTexture;
    setTexture(texture);

    if (rotationDeg != 0.0f)
    {
        pushMatrix();
        translate(dstX + pivotX, dstY + pivotY);
        rotate(rotationDeg, 0.0f, 0.0f, 1.0f);
        translate(-pivotX, -pivotY);
        dstX = 0.0f;
        dstY = 0.0f;
    }

    begin(ModeTriangles);

    float u0 = srcX, v0 = srcY;
    float u1 = srcX + srcW, v1 = srcY + srcH;

    if (srcW == 0.0f || srcH == 0.0f)
    {
        u0 = 0.0f;
        v0 = 0.0f;
        u1 = 1.0f;
        v1 = 1.0f;
    }

    emitTexturedTriangle(dstX, dstY, u0, v0, dstX + dstW, dstY, u1, v0, dstX + dstW, dstY + dstH,
            u1, v1);
    emitTexturedTriangle(dstX, dstY, u0, v0, dstX + dstW, dstY + dstH, u1, v1, dstX, dstY + dstH,
            u0, v1);

    end();

    if (rotationDeg != 0.0f)
    {
        popMatrix();
    }
    setTexture(previous);
}


unsigned int BatchRenderer::packColor(unsigned char r, unsigned char g, unsigned char b,
        unsigned char a)
{
    return (unsigned int) r | ((unsigned int) g << 8) | ((unsigned int) b << 16) |
           ((unsigned int) a << 24);
}

void BatchRenderer::unpackColor(unsigned int packed, unsigned char& r, unsigned char& g,
        unsigned char& b, unsigned char& a)
{
    r = packed & 0xFF;
    g = (packed >> 8) & 0xFF;
    b = (packed >> 16) & 0xFF;
    a = (packed >> 24) & 0xFF;
}


void BatchRenderer::submitVertex(float x, float y, float z)
{
    submitVertex(x, y, z, mCurrentTexcoord[0], mCurrentTexcoord[1]);
}

void BatchRenderer::submitVertex(float x, float y, float z, float u, float v)
{
    Vertex vert;
    vert.x = x;
    vert.y = y;
    vert.z = z;
    vert.u = u;
    vert.v = v;
    unpackColor(mCurrentColor, vert.r, vert.g, vert.b, vert.a);
    mVertices.push_back(vert);
}

void BatchRenderer::applyTransform(float& x, float& y, float& z)
{
    const Math::Vec4 transformed = mCurrentMatrix * Math::Vec4(x, y, z, 1.0f);
    x = transformed.x;
    y = transformed.y;
    z = transformed.z;
}

Math::Vec4 fontGlyphUVRect(unsigned char code)
{
    if (code < 32 || code > 127 || code == ' ') return Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
    const float cw = 8.f / (float) FONT_ATLAS_W;
    const float ch = 8.f / (float) FONT_ATLAS_H;
    const int g = code - 32;
    const float u0 = (float) (g % FONT_COLS) * cw;
    const float v0 = (float) (g / FONT_COLS) * ch;
    return Math::Vec4(u0, v0, cw, ch);
}

void BatchRenderer::drawText(float x, float y, float size, const char* text)
{
    if (!text || size <= 0.f) return;

    const prisma::TextureHandle previous = mCurrentTexture;
    setTexture(mFontTexture);
    begin(ModeTriangles);

    float penX = x, penY = y;

    for (const char* c = text; *c; ++c)
    {
        if (*c == '\n')
        {
            penX = x;
            penY += size;
            continue;
        }
        std::uint8_t code = (std::uint8_t) *c;
        if (code < 32 || code > 127) code = '?';
        const Math::Vec4 rect = fontGlyphUVRect(code);
        if (rect.z > 0.0f)
        {
            const float u0 = rect.x, v0 = rect.y, u1 = rect.x + rect.z, v1 = rect.y + rect.w;
            emitTexturedTriangle(penX, penY, u0, v0, penX + size, penY, u1, v0, penX, penY + size,
                    u0, v1);
            emitTexturedTriangle(penX + size, penY, u1, v0, penX + size, penY + size, u1, v1, penX,
                    penY + size, u0, v1);
        }
        penX += size;
    }

    end();
    setTexture(previous);
}

float BatchRenderer::textWidth(float size, const char* text) const
{
    if (!text) return 0.f;
    std::uint32_t longest = 0, line = 0;
    for (const char* c = text; *c; ++c)
    {
        if (*c == '\n')
        {
            if (line > longest) longest = line;
            line = 0;
            continue;
        }
        ++line;
    }
    if (line > longest) longest = line;
    return (float) longest * size;
}

} // namespace prisma3d
