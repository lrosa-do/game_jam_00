#include "prisma3d/render/TextureStore.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"

#include "Bcn.h"
#include "dds.h"
#include "platform.h"
#include "stb_image.h"

#include <ct/vector.hpp>

#include <string.h>

namespace prisma3d
{

namespace
{

bool mapDxgiFormat(dds::DXGI_FORMAT format, bool srgb, DdsInfo* info)
{
    using prisma::TextureFormat;
    switch (format)
    {
        case dds::DXGI_FORMAT_R8G8B8A8_UNORM:
            info->format = srgb ? TextureFormat::RGBA8Srgb : TextureFormat::RGBA8;
            return true;
        case dds::DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            info->format = TextureFormat::RGBA8Srgb;
            return true;
        case dds::DXGI_FORMAT_B8G8R8A8_UNORM:
        case dds::DXGI_FORMAT_B8G8R8X8_UNORM:
            info->format = srgb ? TextureFormat::RGBA8Srgb : TextureFormat::RGBA8;
            info->swapRedBlue = true;
            info->opaqueAlpha = format == dds::DXGI_FORMAT_B8G8R8X8_UNORM;
            return true;
        case dds::DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
            info->format = TextureFormat::RGBA8Srgb;
            info->swapRedBlue = true;
            return true;
        case dds::DXGI_FORMAT_R8_UNORM:
            info->format = TextureFormat::R8;
            return true;
        case dds::DXGI_FORMAT_R8G8_UNORM:
            info->format = TextureFormat::RG8;
            return true;
        case dds::DXGI_FORMAT_R16_FLOAT:
            info->format = TextureFormat::R16F;
            return true;
        case dds::DXGI_FORMAT_R16G16_FLOAT:
            info->format = TextureFormat::RG16F;
            return true;
        case dds::DXGI_FORMAT_R16G16B16A16_FLOAT:
            info->format = TextureFormat::RGBA16F;
            return true;
        case dds::DXGI_FORMAT_R32_FLOAT:
            info->format = TextureFormat::R32F;
            return true;
        case dds::DXGI_FORMAT_R32G32_FLOAT:
            info->format = TextureFormat::RG32F;
            return true;
        case dds::DXGI_FORMAT_R32G32B32A32_FLOAT:
            info->format = TextureFormat::RGBA32F;
            return true;
        case dds::DXGI_FORMAT_R32_UINT:
            info->format = TextureFormat::R32UInt;
            return true;
        case dds::DXGI_FORMAT_R10G10B10A2_UNORM:
            info->format = TextureFormat::RGB10A2;
            return true;
        case dds::DXGI_FORMAT_R11G11B10_FLOAT:
            info->format = TextureFormat::R11G11B10F;
            return true;
        case dds::DXGI_FORMAT_BC1_UNORM:
            info->format = srgb ? TextureFormat::BC1Srgb : TextureFormat::BC1;
            return true;
        case dds::DXGI_FORMAT_BC1_UNORM_SRGB:
            info->format = TextureFormat::BC1Srgb;
            return true;
        case dds::DXGI_FORMAT_BC2_UNORM:
            info->format = srgb ? TextureFormat::BC2Srgb : TextureFormat::BC2;
            return true;
        case dds::DXGI_FORMAT_BC2_UNORM_SRGB:
            info->format = TextureFormat::BC2Srgb;
            return true;
        case dds::DXGI_FORMAT_BC3_UNORM:
            info->format = srgb ? TextureFormat::BC3Srgb : TextureFormat::BC3;
            return true;
        case dds::DXGI_FORMAT_BC3_UNORM_SRGB:
            info->format = TextureFormat::BC3Srgb;
            return true;
        case dds::DXGI_FORMAT_BC4_UNORM:
            info->format = TextureFormat::BC4;
            return true;
        case dds::DXGI_FORMAT_BC5_UNORM:
            info->format = TextureFormat::BC5;
            return true;
        case dds::DXGI_FORMAT_BC6H_UF16:
            info->format = TextureFormat::BC6H;
            return true;
        case dds::DXGI_FORMAT_BC7_UNORM:
            info->format = srgb ? TextureFormat::BC7Srgb : TextureFormat::BC7;
            return true;
        case dds::DXGI_FORMAT_BC7_UNORM_SRGB:
            info->format = TextureFormat::BC7Srgb;
            return true;
        default:
            return false;
    }
}

typedef ct::Handle32<TextureInfo> SlotKey;

SlotKey toKey(TextureId id)
{
    return SlotKey::from_bits(id.bits());
}

TextureId toId(SlotKey key)
{
    return TextureId::from_bits(key.bits());
}

bool endsWith(const char* text, const char* suffix)
{
    const size_t length = strlen(text);
    const size_t tail = strlen(suffix);
    if (tail > length) return false;
    for (size_t i = 0; i < tail; ++i)
    {
        char a = text[length - tail + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a + 32);
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b + 32);
        if (a != b) return false;
    }
    return true;
}

std::uint32_t fullMipCount(std::uint32_t width, std::uint32_t height)
{
    std::uint32_t size = width > height ? width : height;
    std::uint32_t levels = 1;
    while (size > 1)
    {
        size >>= 1;
        ++levels;
    }
    return levels;
}

prisma::TextureHandle createDdsTexture(prisma::Driver* driver, const std::uint8_t* data,
        std::size_t size, bool srgb, bool generateMips, const char* name, unsigned skipMips,
        prisma::TextureDesc* used)
{
    DdsInfo info;
    if (!parseDds(data, size, srgb, &info))
    {
        log_error("texture: invalid or unsupported dds");
        return prisma::TextureHandle();
    }
    const prisma::Caps& caps = driver->caps();
    const prisma::TextureFormat format = info.format;
    if (info.type == prisma::TextureType::TextureCubeArray && !caps.cubeArrays)
    {
        log_error("texture: cube arrays are not supported by the driver");
        return prisma::TextureHandle();
    }

    const dds::Header header = dds::read_header(data, size);

    const bool isBc1 =
            format == prisma::TextureFormat::BC1 || format == prisma::TextureFormat::BC1Srgb;
    const bool isBc2 =
            format == prisma::TextureFormat::BC2 || format == prisma::TextureFormat::BC2Srgb;
    const bool isBc3 =
            format == prisma::TextureFormat::BC3 || format == prisma::TextureFormat::BC3Srgb;
    if ((isBc1 || isBc2 || isBc3) && info.layers == 1 &&
            info.type == prisma::TextureType::Texture2D &&
            ((generateMips && info.mipLevels == 1) || !caps.textureBC))
    {
        const bool srgbFormat = format == prisma::TextureFormat::BC1Srgb ||
                                format == prisma::TextureFormat::BC2Srgb ||
                                format == prisma::TextureFormat::BC3Srgb;
        ct::Vector<std::uint8_t> pixels;
        pixels.resize(static_cast<size_t>(info.width) * info.height * 4);
        if (decodeBcn(isBc1   ? BcnKind::BC1
                      : isBc2 ? BcnKind::BC2
                              : BcnKind::BC3,
                    data + header.mip_offset(0, 0), static_cast<size_t>(header.mip_size(0)),
                    info.width, info.height, pixels.data()))
        {
            prisma::TextureDesc decoded;
            decoded.format =
                    srgbFormat ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
            decoded.width = info.width;
            decoded.height = info.height;
            decoded.mipLevels = generateMips ? 0 : 1;
            decoded.generateMipmaps = generateMips;
            decoded.data = pixels.data();
            decoded.debugName = name;
            *used = decoded;
            const prisma::TextureHandle texture = driver->createTexture(decoded);
            if (!texture.valid()) log_error("texture: the driver could not create the texture");
            return texture;
        }
    }

    if (isBlockCompressed(format) && !caps.textureBC)
    {
        log_error("texture: block compression is not supported by the driver");
        return prisma::TextureHandle();
    }

    prisma::TextureDesc desc;
    const unsigned skip = info.type == prisma::TextureType::Texture2D && info.mipLevels > 1
                                  ? (skipMips < info.mipLevels - 1 ? skipMips : info.mipLevels - 1)
                                  : 0;
    desc.type = info.type;
    desc.format = format;
    desc.width = info.width >> skip > 0 ? info.width >> skip : 1;
    desc.height = info.height >> skip > 0 ? info.height >> skip : 1;
    desc.depth = info.type == prisma::TextureType::Texture3D          ? info.depth
                 : info.type == prisma::TextureType::TextureCubeArray ? info.layers / 6
                                                                      : info.layers;
    desc.mipLevels = info.mipLevels - skip;
    desc.debugName = name;

    const bool autoMips = generateMips && info.mipLevels == 1 && !isBlockCompressed(format) &&
                          info.type == prisma::TextureType::Texture2D &&
                          format != prisma::TextureFormat::R32UInt;
    if (autoMips)
    {
        desc.mipLevels = 0;
        desc.generateMipmaps = true;
        desc.data = data + header.mip_offset(0, 0);
        ct::Vector<std::uint8_t> swapped;
        if (info.swapRedBlue)
        {
            const size_t bytes = static_cast<size_t>(header.mip_size(0));
            swapped.resize(bytes);
            for (size_t i = 0; i + 3 < bytes; i += 4)
            {
                swapped[i] = static_cast<const std::uint8_t*>(desc.data)[i + 2];
                swapped[i + 1] = static_cast<const std::uint8_t*>(desc.data)[i + 1];
                swapped[i + 2] = static_cast<const std::uint8_t*>(desc.data)[i];
                swapped[i + 3] =
                        info.opaqueAlpha ? 255 : static_cast<const std::uint8_t*>(desc.data)[i + 3];
            }
            desc.data = swapped.data();
        }
        *used = desc;
        const prisma::TextureHandle texture = driver->createTexture(desc);
        if (!texture.valid()) log_error("texture: the driver could not create the texture");
        return texture;
    }

    *used = desc;
    const prisma::TextureHandle texture = driver->createTexture(desc);
    if (!texture.valid())
    {
        log_error("texture: the driver could not create the texture");
        return texture;
    }
    ct::Vector<std::uint8_t> converted;
    for (unsigned layer = 0; layer < info.layers; ++layer)
    {
        for (unsigned mip = 0; mip < info.mipLevels - skip; ++mip)
        {
            const std::uint8_t* source = data + header.mip_offset(mip + skip, layer);
            if (info.swapRedBlue)
            {
                const size_t bytes = static_cast<size_t>(header.mip_size(mip + skip));
                converted.resize(bytes);
                for (size_t i = 0; i + 3 < bytes; i += 4)
                {
                    converted[i] = source[i + 2];
                    converted[i + 1] = source[i + 1];
                    converted[i + 2] = source[i];
                    converted[i + 3] = info.opaqueAlpha ? 255 : source[i + 3];
                }
                source = converted.data();
            }
            driver->updateTexture(texture, mip, layer, source);
        }
    }
    return texture;
}

} // namespace

bool isBlockCompressed(prisma::TextureFormat format)
{
    return format >= prisma::TextureFormat::BC1 && format <= prisma::TextureFormat::BC7Srgb;
}

bool parseDds(const std::uint8_t* data, std::size_t size, bool srgb, DdsInfo* info)
{
    const dds::Header header = dds::read_header(data, size);
    if (!header.is_valid()) return false;
    *info = DdsInfo();
    if (!mapDxgiFormat(header.format(), srgb, info)) return false;

    info->width = header.width();
    info->height = header.height();
    info->depth = header.depth();
    info->mipLevels = header.mip_levels();
    info->layers = header.array_size();
    if (header.is_3d())
    {
        info->type = prisma::TextureType::Texture3D;
        info->layers = 1;
    }
    else
    {
        info->depth = 1;
        if (header.is_cubemap())
        {
            info->type = info->layers > 6 ? prisma::TextureType::TextureCubeArray
                                          : prisma::TextureType::TextureCube;
            if (info->layers % 6 != 0) return false;
        }
        else if (info->layers > 1)
            info->type = prisma::TextureType::Texture2DArray;
    }
    const unsigned last = info->mipLevels - 1;
    const unsigned long long end =
            header.mip_offset(last, info->layers - 1) + header.mip_size(last);
    return end <= size;
}

TextureStore* TextureStore::sSingleton = nullptr;

TextureStore::TextureStore(prisma::Driver* driver)
        : mDriver(driver)
{
    sSingleton = this;
}

TextureStore::~TextureStore()
{
    clear();
    if (sSingleton == this) sSingleton = nullptr;
}

TextureStore* TextureStore::singleton()
{
    return sSingleton;
}

prisma::Driver* TextureStore::driver() const
{
    return mDriver;
}

TextureId TextureStore::insert(prisma::TextureHandle gpu, const prisma::TextureDesc& desc,
        bool srgb, const char* name)
{
    TextureInfo info;
    info.gpu = gpu;
    info.type = desc.type;
    info.format = desc.format;
    info.width = desc.width;
    info.height = desc.height;
    info.depth = desc.depth;
    info.mipLevels = desc.mipLevels == 0 ? fullMipCount(desc.width, desc.height) : desc.mipLevels;
    switch (desc.type)
    {
        case prisma::TextureType::Texture2DArray:
            info.layers = desc.depth;
            break;
        case prisma::TextureType::TextureCube:
            info.layers = 6;
            break;
        case prisma::TextureType::TextureCubeArray:
            info.layers = desc.depth * 6;
            break;
        default:
            info.layers = 1;
            break;
    }
    info.srgb = srgb;
    info.state = TextureState::Ready;
    if (name) info.name = name;
    return toId(mTextures.insert(static_cast<TextureInfo&&>(info)));
}

TextureId TextureStore::create(const prisma::TextureDesc& desc, bool srgb)
{
    const prisma::TextureHandle gpu = mDriver->createTexture(desc);
    if (!gpu.valid()) return TextureId();
    return insert(gpu, desc, srgb, desc.debugName);
}

TextureId TextureStore::createFromDds(const std::uint8_t* data, std::size_t size, bool srgb,
        bool generateMips, const char* name, unsigned skipMips)
{
    prisma::TextureDesc used;
    const prisma::TextureHandle gpu =
            createDdsTexture(mDriver, data, size, srgb, generateMips, name, skipMips, &used);
    if (!gpu.valid()) return TextureId();
    const bool srgbFormat = used.format == prisma::TextureFormat::RGBA8Srgb ||
                            (used.format >= prisma::TextureFormat::BC1Srgb &&
                                    used.format <= prisma::TextureFormat::BC7Srgb);
    return insert(gpu, used, srgb || srgbFormat, name);
}

TextureId TextureStore::createFromImage(const std::uint8_t* data, std::size_t size, bool srgb,
        bool generateMips, bool hdr, const char* name)
{
    int width = 0;
    int height = 0;
    int components = 0;
    prisma::TextureDesc desc;
    desc.debugName = name;
    unsigned char* pixels = nullptr;
    float* floats = nullptr;
    if (hdr)
    {
        floats = stbi_loadf_from_memory(data, static_cast<int>(size), &width, &height, &components,
                4);
        desc.format = prisma::TextureFormat::RGBA32F;
        desc.data = floats;
    }
    else
    {
        pixels = stbi_load_from_memory(data, static_cast<int>(size), &width, &height, &components,
                4);
        desc.format = srgb ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
        desc.data = pixels;
        desc.mipLevels = generateMips ? 0 : 1;
        desc.generateMipmaps = generateMips;
    }
    if (!desc.data)
    {
        log_error("texture: could not decode the image");
        return TextureId();
    }
    desc.width = static_cast<unsigned>(width);
    desc.height = static_cast<unsigned>(height);
    const prisma::TextureHandle gpu = mDriver->createTexture(desc);
    stbi_image_free(pixels);
    stbi_image_free(floats);
    if (!gpu.valid())
    {
        log_error("texture: the driver could not create the texture");
        return TextureId();
    }
    return insert(gpu, desc, srgb, name);
}

TextureId TextureStore::createFromPixmap(const Pixmap& pixmap, bool srgb, bool generateMips,
        const char* name, unsigned maxMipLevels)
{
    if (!pixmap.is_valid() || pixmap.width <= 0 || pixmap.height <= 0)
    {
        log_error("texture: invalid pixmap");
        return TextureId();
    }
    Pixmap* converted = nullptr;
    const Pixmap* source = &pixmap;
    if (pixmap.components != 4)
    {
        converted = pixmap.convert_to_rgba();
        if (!converted || !converted->is_valid())
        {
            delete converted;
            log_error("texture: could not convert the pixmap");
            return TextureId();
        }
        source = converted;
    }
    prisma::TextureDesc desc;
    desc.format = srgb ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
    desc.width = static_cast<unsigned>(source->width);
    desc.height = static_cast<unsigned>(source->height);
    desc.mipLevels = generateMips ? maxMipLevels : 1;
    desc.generateMipmaps = generateMips;
    desc.data = source->pixels;
    desc.debugName = name;
    const prisma::TextureHandle gpu = mDriver->createTexture(desc);
    delete converted;
    if (!gpu.valid())
    {
        log_error("texture: the driver could not create the texture");
        return TextureId();
    }
    return insert(gpu, desc, srgb, name);
}

TextureId TextureStore::createFromFile(const ct::String& path, bool srgb, bool generateMips,
        unsigned skipMips)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty())
    {
        log_error("texture: could not read %s", path.c_str());
        return TextureId();
    }
    TextureId id;
    if (endsWith(path.c_str(), ".dds"))
        id = createFromDds(bytes.data(), bytes.size(), srgb, generateMips, path.c_str(), skipMips);
    else
        id = createFromImage(bytes.data(), bytes.size(), srgb, generateMips,
                endsWith(path.c_str(), ".hdr"), path.c_str());
    if (id.valid()) setPath(id, path);
    return id;
}

TextureId TextureStore::find(const ct::String& path) const
{
    if (path.empty()) return TextureId();
    for (std::size_t i = 0; i < mTextures.size(); ++i)
        if (mTextures.items()[i].path == path) return toId(mTextures.handle_at(i));
    return TextureId();
}

TextureId TextureStore::find(const ct::String& path, bool srgb) const
{
    if (path.empty()) return TextureId();
    const auto items = mTextures.items();
    for (std::size_t i = 0; i < items.size(); ++i)
        if (items[i].path == path && items[i].srgb == srgb) return toId(mTextures.handle_at(i));
    return TextureId();
}

TextureId TextureStore::createEmpty(std::uint32_t width, std::uint32_t height, bool srgb,
        bool generateMips, const char* name)
{
    if (width == 0 || height == 0) return TextureId();
    prisma::TextureDesc desc;
    desc.format = srgb ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
    desc.width = width;
    desc.height = height;
    desc.mipLevels = generateMips ? 0 : 1;
    desc.generateMipmaps = false;
    desc.data = nullptr;
    desc.debugName = name;
    const TextureId id = create(desc, srgb);
    if (id.valid()) setState(id, TextureState::Pending);
    return id;
}

bool TextureStore::uploadRows(TextureId texture, std::uint32_t y, std::uint32_t rows,
        const void* rgba)
{
    const TextureInfo* info = mTextures.get(toKey(texture));
    if (!info || !rgba || rows == 0) return false;
    if (info->type != prisma::TextureType::Texture2D) return false;
    if (info->format != prisma::TextureFormat::RGBA8 &&
            info->format != prisma::TextureFormat::RGBA8Srgb)
        return false;
    if (static_cast<std::uint64_t>(y) + rows > info->height) return false;
    prisma::TextureRegion region;
    region.mip = 0;
    region.layer = 0;
    region.x = 0;
    region.y = y;
    region.width = info->width;
    region.height = rows;
    mDriver->updateTextureRegion(info->gpu, region, rgba);
    return true;
}

bool TextureStore::finish(TextureId texture)
{
    TextureInfo* info = mTextures.get(toKey(texture));
    if (!info) return false;
    if (info->mipLevels > 1) mDriver->generateMipmaps(info->gpu);
    info->state = TextureState::Ready;
    return true;
}

TextureId TextureStore::proxyCreate(TextureId base)
{
    const TextureInfo* source = mTextures.get(toKey(base));
    if (!source || source->proxy) return TextureId();
    TextureInfo info;
    info.type = source->type;
    info.format = source->format;
    info.width = source->width;
    info.height = source->height;
    info.depth = source->depth;
    info.mipLevels = source->mipLevels;
    info.layers = source->layers;
    info.srgb = source->srgb;
    info.state = source->state;
    info.proxy = true;
    info.proxyTo = base;
    return toId(mTextures.insert(static_cast<TextureInfo&&>(info)));
}

bool TextureStore::proxyUpdate(TextureId proxy, TextureId target)
{
    TextureInfo* info = mTextures.get(toKey(proxy));
    if (!info || !info->proxy) return false;
    const TextureInfo* source = mTextures.get(toKey(target));
    if (!source || source->proxy) return false;
    info->type = source->type;
    info->format = source->format;
    info->width = source->width;
    info->height = source->height;
    info->depth = source->depth;
    info->mipLevels = source->mipLevels;
    info->layers = source->layers;
    info->srgb = source->srgb;
    info->state = source->state;
    info->proxyTo = target;
    return true;
}

bool TextureStore::isProxy(TextureId texture) const
{
    const TextureInfo* info = mTextures.get(toKey(texture));
    return info && info->proxy;
}

TextureId TextureStore::proxyTarget(TextureId texture) const
{
    const TextureInfo* info = mTextures.get(toKey(texture));
    if (!info || !info->proxy) return TextureId();
    return mTextures.contains(toKey(info->proxyTo)) ? info->proxyTo : TextureId();
}

void TextureStore::setSequence(TextureId texture, const ct::Vector<ct::String>& paths, float fps)
{
    TextureInfo* info = mTextures.get(toKey(texture));
    if (!info) return;
    info->sequence = paths;
    info->sequenceFps = fps;
}

bool TextureStore::replace(TextureId texture, TextureId by)
{
    if (texture == by) return false;
    TextureInfo* target = mTextures.get(toKey(texture));
    TextureInfo* source = mTextures.get(toKey(by));
    if (!target || !source || target->proxy || source->proxy) return false;
    mDriver->destroy(target->gpu);
    target->gpu = source->gpu;
    target->type = source->type;
    target->format = source->format;
    target->width = source->width;
    target->height = source->height;
    target->depth = source->depth;
    target->mipLevels = source->mipLevels;
    target->layers = source->layers;
    target->srgb = source->srgb;
    target->state = TextureState::Ready;
    if (target->path.empty()) target->path = source->path;
    mTextures.erase(toKey(by));
    return true;
}

void TextureStore::destroy(TextureId texture)
{
    TextureInfo* info = mTextures.get(toKey(texture));
    if (!info) return;
    if (!info->proxy) mDriver->destroy(info->gpu);
    mTextures.erase(toKey(texture));
    if (texture == mWhite) mWhite = TextureId();
    if (texture == mBlack) mBlack = TextureId();
    if (texture == mNormal) mNormal = TextureId();
    if (texture == mPlaceholder) mPlaceholder = TextureId();
}

void TextureStore::clear()
{
    for (TextureInfo& info: mTextures)
        if (!info.proxy) mDriver->destroy(info.gpu);
    mTextures.clear();
    mWhite = TextureId();
    mBlack = TextureId();
    mNormal = TextureId();
    mPlaceholder = TextureId();
}

bool TextureStore::valid(TextureId texture) const
{
    const TextureInfo* info = mTextures.get(toKey(texture));
    if (!info) return false;
    return !info->proxy || mTextures.contains(toKey(info->proxyTo));
}

const TextureInfo* TextureStore::get(TextureId texture) const
{
    return mTextures.get(toKey(texture));
}

prisma::TextureHandle TextureStore::gpu(TextureId texture) const
{
    const TextureInfo* info = mTextures.get(toKey(texture));
    if (!info) return prisma::TextureHandle();
    if (!info->proxy) return info->gpu;
    const TextureInfo* target = mTextures.get(toKey(info->proxyTo));
    return target ? target->gpu : prisma::TextureHandle();
}

void TextureStore::setPath(TextureId texture, const ct::String& path)
{
    TextureInfo* info = mTextures.get(toKey(texture));
    if (info) info->path = path;
}

void TextureStore::setState(TextureId texture, TextureState state)
{
    TextureInfo* info = mTextures.get(toKey(texture));
    if (info) info->state = state;
}

std::size_t TextureStore::count() const
{
    return mTextures.size();
}

TextureId TextureStore::solid(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a,
        const char* name)
{
    const std::uint8_t pixel[4] = { r, g, b, a };
    prisma::TextureDesc desc;
    desc.format = prisma::TextureFormat::RGBA8;
    desc.width = 1;
    desc.height = 1;
    desc.mipLevels = 1;
    desc.data = pixel;
    desc.debugName = name;
    return create(desc, false);
}

TextureId TextureStore::white()
{
    if (!mTextures.contains(toKey(mWhite))) mWhite = solid(255, 255, 255, 255, "default.white");
    return mWhite;
}

TextureId TextureStore::black()
{
    if (!mTextures.contains(toKey(mBlack))) mBlack = solid(0, 0, 0, 255, "default.black");
    return mBlack;
}

TextureId TextureStore::normal()
{
    if (!mTextures.contains(toKey(mNormal))) mNormal = solid(128, 128, 255, 255, "default.normal");
    return mNormal;
}

TextureId TextureStore::placeholder()
{
    if (!mTextures.contains(toKey(mPlaceholder)))
        mPlaceholder = solid(255, 0, 255, 255, "default.placeholder");
    return mPlaceholder;
}

} // namespace prisma3d
