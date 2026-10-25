#pragma once

#include "prisma/rhi/Driver.h"

#include <ct/slotmap32.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class Pixmap;

struct TextureTag;
using TextureId = ct::Handle32<TextureTag>;

enum class TextureState : std::uint8_t
{
    Ready,
    Pending,
    Failed
};

struct TextureInfo
{
    prisma::TextureHandle gpu;
    prisma::TextureType type = prisma::TextureType::Texture2D;
    prisma::TextureFormat format = prisma::TextureFormat::RGBA8;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t depth = 1;
    std::uint32_t mipLevels = 1;
    std::uint32_t layers = 1;
    bool srgb = false;
    TextureState state = TextureState::Ready;
    ct::String name;
    ct::String path;
    bool proxy = false;
    TextureId proxyTo;
    ct::Vector<ct::String> sequence;
    float sequenceFps = 0.0f;
};

struct DdsInfo
{
    prisma::TextureType type = prisma::TextureType::Texture2D;
    prisma::TextureFormat format = prisma::TextureFormat::RGBA8;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t depth = 1;
    std::uint32_t mipLevels = 1;
    std::uint32_t layers = 1;
    bool swapRedBlue = false;
    bool opaqueAlpha = false;
};

bool parseDds(const std::uint8_t* data, std::size_t size, bool srgb, DdsInfo* info);
bool isBlockCompressed(prisma::TextureFormat format);

class TextureStore
{
public:
    explicit TextureStore(prisma::Driver* driver);
    ~TextureStore();
    TextureStore(const TextureStore&) = delete;
    TextureStore& operator=(const TextureStore&) = delete;

    static TextureStore* singleton();
    prisma::Driver* driver() const;

    TextureId create(const prisma::TextureDesc& desc, bool srgb = false);
    TextureId createFromDds(const std::uint8_t* data, std::size_t size, bool srgb,
            bool generateMips, const char* name = nullptr, unsigned skipMips = 0);
    TextureId createFromImage(const std::uint8_t* data, std::size_t size, bool srgb,
            bool generateMips, bool hdr, const char* name = nullptr);
    TextureId createFromPixmap(const Pixmap& pixmap, bool srgb, bool generateMips,
            const char* name = nullptr, unsigned maxMipLevels = 0);
    TextureId createFromFile(const ct::String& path, bool srgb, bool generateMips,
            unsigned skipMips = 0);
    TextureId find(const ct::String& path) const;
    TextureId find(const ct::String& path, bool srgb) const;

    TextureId createEmpty(std::uint32_t width, std::uint32_t height, bool srgb, bool generateMips,
            const char* name = nullptr);
    bool uploadRows(TextureId texture, std::uint32_t y, std::uint32_t rows, const void* rgba);
    bool finish(TextureId texture);

    TextureId proxyCreate(TextureId base);
    bool proxyUpdate(TextureId proxy, TextureId target);
    bool isProxy(TextureId texture) const;
    TextureId proxyTarget(TextureId texture) const;
    void setSequence(TextureId texture, const ct::Vector<ct::String>& paths, float fps);

    bool replace(TextureId texture, TextureId by);
    void destroy(TextureId texture);
    void clear();

    bool valid(TextureId texture) const;
    const TextureInfo* get(TextureId texture) const;
    prisma::TextureHandle gpu(TextureId texture) const;
    void setPath(TextureId texture, const ct::String& path);
    void setState(TextureId texture, TextureState state);
    std::size_t count() const;

    TextureId white();
    TextureId black();
    TextureId normal();
    TextureId placeholder();

private:
    static TextureStore* sSingleton;

    TextureId insert(prisma::TextureHandle gpu, const prisma::TextureDesc& desc, bool srgb,
            const char* name);
    TextureId solid(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a,
            const char* name);

    prisma::Driver* mDriver;
    ct::SlotMap32<TextureInfo> mTextures;
    TextureId mWhite;
    TextureId mBlack;
    TextureId mNormal;
    TextureId mPlaceholder;
};

} // namespace prisma3d
