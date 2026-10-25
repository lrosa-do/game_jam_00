#pragma once

#include "prisma3d/render/TextureStore.h"

#include <igui/Backend.hpp>
#include <igui/FontAtlas.hpp>

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class GuiBackend : public ig::Backend
{
public:
    GuiBackend(prisma::Driver* driver, TextureStore& textures);
    ~GuiBackend() override;
    GuiBackend(const GuiBackend&) = delete;
    GuiBackend& operator=(const GuiBackend&) = delete;

    bool init();
    bool ready() const;
    ig::FontAtlas& fontAtlas();
    TextureId fontTexture() const;

    static ig::TextureId guiTexture(TextureId texture);
    static TextureId engineTexture(ig::TextureId texture);

    ig::TextMetrics measureText(ig::FontId font, ig::StringView text, float logicalSize,
            float dpiScale) override;
    bool render(const ig::DrawData& data) override;
    ig::String clipboardText() override;
    bool setClipboardText(ig::StringView text) override;

    std::uint32_t drawCalls() const;

private:
    bool ensureBuffers(std::uint32_t vertexBytes, std::uint32_t indexBytes);

    prisma::Driver* mDriver;
    TextureStore& mTextures;
    ig::FontAtlas mAtlas;
    TextureId mFontTexture;
    prisma::ShaderHandle mVertexShader;
    prisma::ShaderHandle mFragmentShader;
    prisma::PipelineHandle mPipeline;
    prisma::SamplerHandle mSampler;
    prisma::BufferHandle mVertexBuffer;
    prisma::BufferHandle mIndexBuffer;
    prisma::BufferHandle mUniformBuffer;
    std::uint32_t mVertexCapacity;
    std::uint32_t mIndexCapacity;
    std::uint32_t mDrawCalls;
    bool mTextWarned;
};

} // namespace prisma3d
