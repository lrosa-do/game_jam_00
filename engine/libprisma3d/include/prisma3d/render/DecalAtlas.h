#pragma once

#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class DecalAtlas
{
public:
    struct Item
    {
        TextureId texture;
        std::uint32_t pixelWidth = 0;
        std::uint32_t pixelHeight = 0;
        int width = 0;
        int height = 0;
        int x = 0;
        int y = 0;
    };

    static void pack(ct::Vector<Item>& items, int mipmaps, std::uint32_t& width,
            std::uint32_t& height);
    static Math::Vec4 itemRect(const Item& item, int mipmaps, std::uint32_t width,
            std::uint32_t height);

    DecalAtlas();
    ~DecalAtlas();
    DecalAtlas(const DecalAtlas&) = delete;
    DecalAtlas& operator=(const DecalAtlas&) = delete;

    bool create(prisma::Driver* driver, TextureStore* textures, int mipmaps = 5);
    void destroy();

    void addTexture(TextureId texture);
    void removeTexture(TextureId texture);
    void markDirty(TextureId texture);
    bool hasTexture(TextureId texture) const;
    std::size_t getTextureCount() const;
    Math::Vec4 getTextureRect(TextureId texture) const;

    bool update();
    bool blit(prisma::TextureHandle source, const Math::Vec4& sourceRect,
            prisma::TextureHandle destination, bool destinationSrgb, std::uint32_t mip,
            std::uint32_t width, std::uint32_t height);

    prisma::TextureHandle getTexture() const;
    prisma::TextureHandle getTextureSrgb() const;
    std::uint32_t getWidth() const;
    std::uint32_t getHeight() const;
    int getMipmaps() const;
    bool isDirty() const;

private:
    struct Entry
    {
        TextureId texture;
        std::uint32_t users = 0;
        Math::Vec4 rect = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        bool placed = false;
    };

    Entry* find(TextureId texture);
    const Entry* find(TextureId texture) const;
    void destroyTextures();
    void fill(prisma::TextureHandle target, bool srgb);

    prisma::Driver* mDriver;
    TextureStore* mTextures;
    ct::Vector<Entry> mEntries;
    prisma::ShaderHandle mVertex;
    prisma::ShaderHandle mFragment;
    prisma::PipelineHandle mPipeline;
    prisma::PipelineHandle mPipelineSrgb;
    prisma::BufferHandle mParams;
    prisma::SamplerHandle mSampler;
    prisma::TextureHandle mTexture;
    prisma::TextureHandle mTextureSrgb;
    std::uint32_t mWidth;
    std::uint32_t mHeight;
    int mMipmaps;
    bool mDirty;
};

} // namespace prisma3d
