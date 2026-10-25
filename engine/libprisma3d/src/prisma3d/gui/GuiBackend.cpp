#include "prisma3d/gui/GuiBackend.h"
#include "prisma3d/render/ShaderVariants.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/RenderProfiler.h"

#include "platform.h"

#include <igui/DrawData.hpp>

#include <math.h>
#include <stddef.h>
#include <string.h>

namespace prisma3d
{

namespace
{

static_assert(sizeof(ig::DrawVertex) == 20, "unexpected vertex layout");

const std::uint32_t kVertexStride = sizeof(ig::DrawVertex);
const std::uint32_t kUniformSize = 16;

struct GuiFrame
{
    float invDisplay[2];
    float pad[2];
};

std::int32_t floorInt(float value)
{
    return static_cast<std::int32_t>(floorf(value));
}

std::int32_t ceilInt(float value)
{
    return static_cast<std::int32_t>(ceilf(value));
}

std::int32_t clampInt(std::int32_t value, std::int32_t low, std::int32_t high)
{
    return value < low ? low : (value > high ? high : value);
}

} // namespace

GuiBackend::GuiBackend(prisma::Driver* driver, TextureStore& textures)
        : mDriver(driver),
          mTextures(textures),
          mAtlas(),
          mVertexCapacity(0),
          mIndexCapacity(0),
          mDrawCalls(0),
          mTextWarned(false)
{
}

GuiBackend::~GuiBackend()
{
    if (!mDriver) return;
    if (mVertexBuffer.valid()) mDriver->destroy(mVertexBuffer);
    if (mIndexBuffer.valid()) mDriver->destroy(mIndexBuffer);
    if (mUniformBuffer.valid()) mDriver->destroy(mUniformBuffer);
    if (mPipeline.valid()) mDriver->destroy(mPipeline);
    if (mSampler.valid()) mDriver->destroy(mSampler);
    if (mVertexShader.valid()) mDriver->destroy(mVertexShader);
    if (mFragmentShader.valid()) mDriver->destroy(mFragmentShader);
    if (mFontTexture.valid()) mTextures.destroy(mFontTexture);
}

bool GuiBackend::init()
{
    if (!mDriver || !mAtlas.valid())
    {
        log_error("gui: the driver or the font atlas is not available");
        return false;
    }

    const prisma::Caps& caps = mDriver->caps();
    mVertexShader = mDriver->createShader(prisma3d::shaderDesc(gui_vert, caps));
    mFragmentShader = mDriver->createShader(prisma3d::shaderDesc(gui_frag, caps));
    if (!mVertexShader.valid() || !mFragmentShader.valid())
    {
        log_error("gui: could not create the shaders");
        return false;
    }

    prisma::PipelineDesc pipeline;
    pipeline.vertexShader = mVertexShader;
    pipeline.fragmentShader = mFragmentShader;
    pipeline.vertexBuffers[0].stride = kVertexStride;
    pipeline.vertexBufferCount = 1;
    pipeline.attributeCount = 3;
    pipeline.attributes[0].location = 0;
    pipeline.attributes[0].format = prisma::VertexFormat::Float2;
    pipeline.attributes[0].offset = static_cast<std::uint32_t>(offsetof(ig::DrawVertex, position));
    pipeline.attributes[1].location = 1;
    pipeline.attributes[1].format = prisma::VertexFormat::Float2;
    pipeline.attributes[1].offset = static_cast<std::uint32_t>(offsetof(ig::DrawVertex, uv));
    pipeline.attributes[2].location = 2;
    pipeline.attributes[2].format = prisma::VertexFormat::UByte4Norm;
    pipeline.attributes[2].offset = static_cast<std::uint32_t>(offsetof(ig::DrawVertex, color));
    pipeline.topology = prisma::Topology::Triangles;
    pipeline.uniformBlockCount = 1;
    pipeline.uniformBlocks[0].name = "Frame";
    pipeline.uniformBlocks[0].slot = 0;
    pipeline.textureCount = 1;
    pipeline.textures[0].name = "uTexture";
    pipeline.textures[0].slot = 0;
    pipeline.depthTest = false;
    pipeline.depthWrite = false;
    pipeline.cullMode = prisma::CullMode::None;
    pipeline.blend = true;
    pipeline.srcColor = prisma::BlendFactor::SrcAlpha;
    pipeline.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
    pipeline.srcAlpha = prisma::BlendFactor::One;
    pipeline.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
    pipeline.colorBlendOp = prisma::BlendOp::Add;
    pipeline.alphaBlendOp = prisma::BlendOp::Add;
    pipeline.debugName = "gui";
    mPipeline = mDriver->createPipeline(pipeline);
    if (!mPipeline.valid())
    {
        log_error("gui: could not create the pipeline");
        return false;
    }

    prisma::SamplerDesc sampler;
    sampler.minFilter = prisma::Filter::Linear;
    sampler.magFilter = prisma::Filter::Linear;
    sampler.mipFilter = prisma::MipFilter::None;
    sampler.addressU = prisma::AddressMode::ClampToEdge;
    sampler.addressV = prisma::AddressMode::ClampToEdge;
    sampler.addressW = prisma::AddressMode::ClampToEdge;
    sampler.debugName = "gui sampler";
    mSampler = mDriver->createSampler(sampler);
    if (!mSampler.valid())
    {
        log_error("gui: could not create the sampler");
        return false;
    }

    prisma::BufferDesc uniform;
    uniform.usage = prisma::BufferUsage::Uniform;
    uniform.update = prisma::BufferUpdate::Dynamic;
    uniform.size = kUniformSize;
    uniform.debugName = "gui uniforms";
    mUniformBuffer = mDriver->createBuffer(uniform);
    if (!mUniformBuffer.valid())
    {
        log_error("gui: could not create the uniform buffer");
        return false;
    }

    const std::uint32_t width = mAtlas.width();
    const std::uint32_t height = mAtlas.height();
    const ct::Span<const std::uint8_t> alpha = mAtlas.pixels();
    const std::size_t count = static_cast<std::size_t>(width) * height;
    if (alpha.size() < count)
    {
        log_error("gui: the font atlas has no pixels");
        return false;
    }
    ct::Vector<std::uint8_t> rgba;
    rgba.resize(count * 4);
    for (std::size_t i = 0; i < count; ++i)
    {
        rgba[i * 4 + 0] = 255;
        rgba[i * 4 + 1] = 255;
        rgba[i * 4 + 2] = 255;
        rgba[i * 4 + 3] = alpha[i];
    }
    prisma::TextureDesc texture;
    texture.format = prisma::TextureFormat::RGBA8;
    texture.width = width;
    texture.height = height;
    texture.mipLevels = 1;
    texture.data = rgba.data();
    texture.debugName = "gui.font";
    mFontTexture = mTextures.create(texture, false);
    if (!mFontTexture.valid())
    {
        log_error("gui: could not create the font texture");
        return false;
    }
    mAtlas.setTexture(guiTexture(mFontTexture));
    return true;
}

bool GuiBackend::ready() const
{
    return mPipeline.valid();
}

ig::FontAtlas& GuiBackend::fontAtlas()
{
    return mAtlas;
}

TextureId GuiBackend::fontTexture() const
{
    return mFontTexture;
}

ig::TextureId GuiBackend::guiTexture(TextureId texture)
{
    return ig::TextureId(texture.valid() ? static_cast<std::uint64_t>(texture.bits()) + 1u : 0u);
}

TextureId GuiBackend::engineTexture(ig::TextureId texture)
{
    if (texture.value == 0) return TextureId();
    return TextureId::from_bits(static_cast<std::uint32_t>(texture.value - 1u));
}

ig::TextMetrics GuiBackend::measureText(ig::FontId font, ig::StringView text, float logicalSize,
        float dpiScale)
{
    (void) dpiScale;
    return mAtlas.measureText(font, text, logicalSize);
}

bool GuiBackend::ensureBuffers(std::uint32_t vertexBytes, std::uint32_t indexBytes)
{
    if (!mVertexBuffer.valid() || mVertexCapacity < vertexBytes)
    {
        std::uint32_t grown = mVertexCapacity * 2;
        if (grown < vertexBytes) grown = vertexBytes;
        if (mVertexBuffer.valid()) mDriver->destroy(mVertexBuffer);
        mVertexBuffer = prisma::BufferHandle();
        mVertexCapacity = 0;
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Vertex;
        desc.update = prisma::BufferUpdate::Stream;
        desc.size = grown;
        desc.debugName = "gui vertices";
        mVertexBuffer = mDriver->createBuffer(desc);
        if (!mVertexBuffer.valid())
        {
            log_error("gui: could not create the vertex buffer");
            return false;
        }
        mVertexCapacity = grown;
    }

    if (!mIndexBuffer.valid() || mIndexCapacity < indexBytes)
    {
        std::uint32_t grown = mIndexCapacity * 2;
        if (grown < indexBytes) grown = indexBytes;
        if (mIndexBuffer.valid()) mDriver->destroy(mIndexBuffer);
        mIndexBuffer = prisma::BufferHandle();
        mIndexCapacity = 0;
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Index;
        desc.update = prisma::BufferUpdate::Stream;
        desc.indexFormat = prisma::IndexFormat::UInt32;
        desc.size = grown;
        desc.debugName = "gui indices";
        mIndexBuffer = mDriver->createBuffer(desc);
        if (!mIndexBuffer.valid())
        {
            log_error("gui: could not create the index buffer");
            return false;
        }
        mIndexCapacity = grown;
    }
    return true;
}

bool GuiBackend::render(const ig::DrawData& data)
{
    mDrawCalls = 0;
    if (data.displaySize.x <= 0.0f || data.displaySize.y <= 0.0f) return true;
    if (data.commands.size() == 0 || data.vertices.size() == 0 || data.indices.size() == 0)
        return true;
    if (!ready()) return false;

    const std::uint32_t vertexBytes =
            static_cast<std::uint32_t>(data.vertices.size() * kVertexStride);
    const std::uint32_t indexBytes =
            static_cast<std::uint32_t>(data.indices.size() * sizeof(ig::DrawIndex));
    if (!ensureBuffers(vertexBytes, indexBytes)) return false;
    mDriver->updateBuffer(mVertexBuffer, 0, data.vertices.data(), vertexBytes);
    mDriver->updateBuffer(mIndexBuffer, 0, data.indices.data(), indexBytes);

    GuiFrame frame;
    frame.invDisplay[0] = 1.0f / data.displaySize.x;
    frame.invDisplay[1] = 1.0f / data.displaySize.y;
    frame.pad[0] = 0.0f;
    frame.pad[1] = 0.0f;
    mDriver->updateBuffer(mUniformBuffer, 0, &frame, sizeof(frame));

    const float scale = data.dpiScale > 0.0f ? data.dpiScale : 1.0f;
    const std::int32_t framebufferWidth =
            static_cast<std::int32_t>(roundf(data.displaySize.x * scale));
    const std::int32_t framebufferHeight =
            static_cast<std::int32_t>(roundf(data.displaySize.y * scale));

    prisma::RenderPassDesc pass;
    pass.colorLoad = prisma::LoadOp::Load;
    pass.depthLoad = prisma::LoadOp::Load;
    pass.depthStore = prisma::StoreOp::Discard;
    GpuProfileScope gpuScope("GUI");
    mDriver->beginRenderPass(pass);

    prisma::Viewport viewport;
    viewport.width = static_cast<float>(framebufferWidth);
    viewport.height = static_cast<float>(framebufferHeight);
    mDriver->setViewport(viewport);
    mDriver->bindPipeline(mPipeline);
    mDriver->bindIndexBuffer(mIndexBuffer);
    mDriver->bindUniformBuffer(0, mUniformBuffer, 0, kUniformSize);

    const TextureId white = mTextures.white();
    for (std::size_t i = 0; i < data.commands.size(); ++i)
    {
        const ig::DrawCommand& command = data.commands[i];
        if (command.type != ig::DrawCommandType::Geometry)
        {
            if (!mTextWarned)
            {
                log_error("gui: text commands are not supported by the backend");
                mTextWarned = true;
            }
            continue;
        }

        const ig::GeometryCommand& geometry = command.payload.geometry;
        const std::int32_t x0 = clampInt(floorInt(geometry.clip.x * scale), 0, framebufferWidth);
        const std::int32_t y0 = clampInt(floorInt(geometry.clip.y * scale), 0, framebufferHeight);
        const std::int32_t x1 = clampInt(ceilInt((geometry.clip.x + geometry.clip.width) * scale),
                0, framebufferWidth);
        const std::int32_t y1 = clampInt(ceilInt((geometry.clip.y + geometry.clip.height) * scale),
                0, framebufferHeight);
        if (x1 <= x0 || y1 <= y0 || geometry.indexCount == 0 || geometry.vertexOffset < 0) continue;

        prisma::Rect scissor;
        scissor.x = x0;
        scissor.y = y0;
        scissor.width = static_cast<std::uint32_t>(x1 - x0);
        scissor.height = static_cast<std::uint32_t>(y1 - y0);
        mDriver->setScissor(scissor);

        TextureId texture = engineTexture(geometry.texture);
        prisma::TextureHandle gpu = mTextures.gpu(texture);
        if (!gpu.valid()) gpu = mTextures.gpu(white);
        mDriver->bindTexture(0, gpu, mSampler);
        mDriver->bindVertexBuffer(0, mVertexBuffer,
                static_cast<std::uint32_t>(geometry.vertexOffset) * kVertexStride);
        mDriver->drawIndexed(geometry.indexCount, geometry.firstIndex);
        ++mDrawCalls;
    }
    mDriver->endRenderPass();
    return true;
}

ig::String GuiBackend::clipboardText()
{
    Device* device = Device::singleton();
    if (!device || !device->clipboardHas()) return ig::String();
    const char* text = device->clipboardGet();
    return text ? ig::String(text, strlen(text)) : ig::String();
}

bool GuiBackend::setClipboardText(ig::StringView text)
{
    Device* device = Device::singleton();
    if (!device) return false;
    const ig::String copy(text.data(), text.size());
    device->clipboardSet(copy.c_str());
    return true;
}

std::uint32_t GuiBackend::drawCalls() const
{
    return mDrawCalls;
}

} // namespace prisma3d
