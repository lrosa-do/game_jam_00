#include "prisma3d/render/DecalAtlas.h"
#include "prisma3d/render/ShaderVariants.h"

#include "platform.h"


namespace prisma3d
{

namespace
{

std::uint32_t nextPowerOfTwo(std::uint32_t value)
{
    std::uint32_t result = 1;
    while (result < value) result <<= 1;
    return result;
}

bool largerFirst(const DecalAtlas::Item& a, const DecalAtlas::Item& b)
{
    if (a.height == b.height) return a.width > b.width;
    return a.height > b.height;
}

} // namespace

void DecalAtlas::pack(ct::Vector<Item>& items, int mipmaps, std::uint32_t& width,
        std::uint32_t& height)
{
    const int border = 1 << mipmaps;
    if (items.empty())
    {
        width = static_cast<std::uint32_t>(border);
        height = static_cast<std::uint32_t>(border);
        return;
    }

    std::uint32_t baseSize = 8;
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        Item& item = items[i];
        item.width = static_cast<int>(item.pixelWidth) / border + 1;
        item.height = static_cast<int>(item.pixelHeight) / border + 1;
        if (baseSize < static_cast<std::uint32_t>(item.width))
            baseSize = nextPowerOfTwo(static_cast<std::uint32_t>(item.width));
    }

    for (std::size_t i = 1; i < items.size(); ++i)
    {
        const Item moving = items[i];
        std::size_t slot = i;
        while (slot > 0 && largerFirst(moving, items[slot - 1]))
        {
            items[slot] = items[slot - 1];
            --slot;
        }
        items[slot] = moving;
    }

    int atlasHeight = 0;
    ct::Vector<int> offsets;
    while (true)
    {
        offsets.clear();
        offsets.resize(baseSize, 0);

        int maxHeight = 2;
        for (std::size_t i = 0; i < items.size(); ++i)
        {
            Item& item = items[i];
            int bestIndex = -1;
            int bestHeight = 0x7FFFFFFF;
            for (std::uint32_t j = 0; j <= baseSize - static_cast<std::uint32_t>(item.width); ++j)
            {
                int columnHeight = 0;
                for (int k = 0; k < item.width; ++k)
                {
                    const int h = offsets[static_cast<std::size_t>(k) + j];
                    if (h > columnHeight)
                    {
                        columnHeight = h;
                        if (columnHeight > bestHeight) break;
                    }
                }
                if (columnHeight < bestHeight)
                {
                    bestHeight = columnHeight;
                    bestIndex = static_cast<int>(j);
                }
            }

            for (int k = 0; k < item.width; ++k)
                offsets[static_cast<std::size_t>(k + bestIndex)] = bestHeight + item.height;

            item.x = bestIndex;
            item.y = bestHeight;
            if (item.y + item.height > maxHeight) maxHeight = item.y + item.height;
        }

        if (static_cast<std::uint32_t>(maxHeight) <= baseSize * 2)
        {
            atlasHeight = maxHeight;
            break;
        }
        baseSize *= 2;
    }

    width = baseSize * static_cast<std::uint32_t>(border);
    height = nextPowerOfTwo(static_cast<std::uint32_t>(atlasHeight * border));
}

Math::Vec4 DecalAtlas::itemRect(const Item& item, int mipmaps, std::uint32_t width,
        std::uint32_t height)
{
    const int border = 1 << mipmaps;
    const float atlasWidth = static_cast<float>(width);
    const float atlasHeight = static_cast<float>(height);
    return Math::Vec4(static_cast<float>(item.x * border + border / 2) / atlasWidth,
            static_cast<float>(item.y * border + border / 2) / atlasHeight,
            static_cast<float>(item.pixelWidth) / atlasWidth,
            static_cast<float>(item.pixelHeight) / atlasHeight);
}

DecalAtlas::DecalAtlas()
        : mDriver(nullptr),
          mTextures(nullptr),
          mEntries(),
          mVertex(),
          mFragment(),
          mPipeline(),
          mPipelineSrgb(),
          mParams(),
          mSampler(),
          mTexture(),
          mTextureSrgb(),
          mWidth(0),
          mHeight(0),
          mMipmaps(5),
          mDirty(true)
{
}

DecalAtlas::~DecalAtlas()
{
    destroy();
}

bool DecalAtlas::create(prisma::Driver* driver, TextureStore* textures, int mipmaps)
{
    destroy();
    mDriver = driver;
    mTextures = textures;
    mMipmaps = mipmaps;
    mDirty = true;
    if (!mDriver || !mTextures) return false;

    const prisma::Caps& caps = mDriver->caps();
    mVertex = mDriver->createShader(prisma3d::shaderDesc(tonemap_vert, caps));
    mFragment = mDriver->createShader(prisma3d::shaderDesc(atlas_copy_frag, caps));

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::Nearest;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "atlas copy";
    mSampler = mDriver->createSampler(samplerDesc);

    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.size = sizeof(float) * 4;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.debugName = "atlas copy params";
    mParams = mDriver->createBuffer(bufferDesc);

    if (!mVertex.valid() || !mFragment.valid() || !mSampler.valid() || !mParams.valid())
    {
        log_error("render: could not create the decal atlas copy resources");
        return false;
    }

    prisma::PipelineDesc desc;
    desc.vertexShader = mVertex;
    desc.fragmentShader = mFragment;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "AtlasCopyParams";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source_color";
    desc.textures[0].slot = 0;
    desc.targets.window = false;
    desc.targets.colors[0] = prisma::TextureFormat::RGBA8;
    desc.targets.colorCount = 1;
    desc.targets.depth = prisma::TextureFormat::None;
    desc.depthTest = false;
    desc.depthWrite = false;
    desc.cullMode = prisma::CullMode::None;
    desc.debugName = "atlas copy";
    mPipeline = mDriver->createPipeline(desc);
    desc.targets.colors[0] = prisma::TextureFormat::RGBA8Srgb;
    desc.debugName = "atlas copy srgb";
    mPipelineSrgb = mDriver->createPipeline(desc);
    if (!mPipeline.valid() || !mPipelineSrgb.valid())
    {
        log_error("render: could not create the decal atlas copy pipelines");
        return false;
    }
    return true;
}

void DecalAtlas::destroyTextures()
{
    if (!mDriver) return;
    if (mTexture.valid()) mDriver->destroy(mTexture);
    if (mTextureSrgb.valid()) mDriver->destroy(mTextureSrgb);
    mTexture = prisma::TextureHandle();
    mTextureSrgb = prisma::TextureHandle();
}

void DecalAtlas::destroy()
{
    if (!mDriver) return;
    destroyTextures();
    if (mPipeline.valid()) mDriver->destroy(mPipeline);
    if (mPipelineSrgb.valid()) mDriver->destroy(mPipelineSrgb);
    if (mParams.valid()) mDriver->destroy(mParams);
    if (mSampler.valid()) mDriver->destroy(mSampler);
    if (mFragment.valid()) mDriver->destroy(mFragment);
    if (mVertex.valid()) mDriver->destroy(mVertex);
    mPipeline = prisma::PipelineHandle();
    mPipelineSrgb = prisma::PipelineHandle();
    mParams = prisma::BufferHandle();
    mSampler = prisma::SamplerHandle();
    mFragment = prisma::ShaderHandle();
    mVertex = prisma::ShaderHandle();
    mEntries.clear();
    mWidth = 0;
    mHeight = 0;
    mDirty = true;
    mDriver = nullptr;
    mTextures = nullptr;
}

DecalAtlas::Entry* DecalAtlas::find(TextureId texture)
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].texture == texture) return &mEntries[i];
    return nullptr;
}

const DecalAtlas::Entry* DecalAtlas::find(TextureId texture) const
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
        if (mEntries[i].texture == texture) return &mEntries[i];
    return nullptr;
}

void DecalAtlas::addTexture(TextureId texture)
{
    if (Entry* entry = find(texture))
    {
        ++entry->users;
        return;
    }
    Entry entry;
    entry.texture = texture;
    entry.users = 1;
    mEntries.push_back(entry);
    mDirty = true;
}

void DecalAtlas::removeTexture(TextureId texture)
{
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        if (!(mEntries[i].texture == texture)) continue;
        if (--mEntries[i].users == 0) mEntries.erase(mEntries.begin() + i);
        return;
    }
}

void DecalAtlas::markDirty(TextureId texture)
{
    if (find(texture)) mDirty = true;
}

bool DecalAtlas::hasTexture(TextureId texture) const
{
    return find(texture) != nullptr;
}

std::size_t DecalAtlas::getTextureCount() const
{
    return mEntries.size();
}

Math::Vec4 DecalAtlas::getTextureRect(TextureId texture) const
{
    const Entry* entry = find(texture);
    return entry ? entry->rect : Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
}

bool DecalAtlas::update()
{
    if (!mDriver || !mTextures || !mPipeline.valid()) return false;

    if (!mDirty)
    {
        for (std::size_t i = 0; i < mEntries.size(); ++i)
        {
            if (mEntries[i].placed) continue;
            const TextureInfo* info = mTextures->get(mEntries[i].texture);
            if (info && info->state == TextureState::Ready && info->gpu.valid()) mDirty = true;
        }
    }
    if (!mDirty) return false;
    mDirty = false;

    destroyTextures();

    ct::Vector<Item> items;
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        Entry& entry = mEntries[i];
        entry.placed = false;
        entry.rect = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        const TextureInfo* info = mTextures->get(entry.texture);
        if (!info || info->state != TextureState::Ready || !info->gpu.valid() || info->width == 0 ||
                info->height == 0)
            continue;
        Item item;
        item.texture = entry.texture;
        item.pixelWidth = info->width;
        item.pixelHeight = info->height;
        items.push_back(item);
    }

    pack(items, mMipmaps, mWidth, mHeight);
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        Entry* entry = find(items[i].texture);
        entry->rect = itemRect(items[i], mMipmaps, mWidth, mHeight);
        entry->placed = true;
    }

    prisma::TextureDesc desc;
    desc.width = mWidth;
    desc.height = mHeight;
    desc.mipLevels = static_cast<std::uint32_t>(mMipmaps);
    desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
    desc.format = prisma::TextureFormat::RGBA8;
    desc.debugName = "decal atlas";
    mTexture = mDriver->createTexture(desc);
    desc.format = prisma::TextureFormat::RGBA8Srgb;
    desc.debugName = "decal atlas srgb";
    mTextureSrgb = mDriver->createTexture(desc);
    if (!mTexture.valid() || !mTextureSrgb.valid())
    {
        log_error("render: could not create the decal atlas textures");
        return false;
    }

    const float full[4] = { 0.0f, 0.0f, 1.0f, 1.0f };
    mDriver->updateBuffer(mParams, 0, full, sizeof(full));
    fill(mTexture, false);
    fill(mTextureSrgb, true);
    return true;
}

void DecalAtlas::fill(prisma::TextureHandle target, bool srgb)
{
    prisma::RenderPassDesc pass;
    pass.colors[0].texture = target;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::Clear;
    pass.clearColor[0] = 0.0f;
    pass.clearColor[1] = 0.0f;
    pass.clearColor[2] = 0.0f;
    pass.clearColor[3] = 0.0f;
    pass.depthLoad = prisma::LoadOp::DontCare;
    pass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);
    mDriver->bindPipeline(srgb ? mPipelineSrgb : mPipeline);
    mDriver->bindUniformBuffer(0, mParams, 0, sizeof(float) * 4);
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        const Entry& entry = mEntries[i];
        if (!entry.placed) continue;
        const TextureInfo* info = mTextures->get(entry.texture);
        prisma::Viewport area;
        area.x = entry.rect.x * static_cast<float>(mWidth);
        area.width = static_cast<float>(info->width);
        area.height = static_cast<float>(info->height);
        area.y = static_cast<float>(mHeight) -
                 (entry.rect.y * static_cast<float>(mHeight) + area.height);
        mDriver->setViewport(area);
        mDriver->bindTexture(0, info->gpu, mSampler);
        mDriver->draw(3, 0);
    }
    mDriver->endRenderPass();
    mDriver->generateMipmaps(target);
}

bool DecalAtlas::blit(prisma::TextureHandle source, const Math::Vec4& sourceRect,
        prisma::TextureHandle destination, bool destinationSrgb, std::uint32_t mip,
        std::uint32_t width, std::uint32_t height)
{
    if (!mDriver || !mPipeline.valid() || !source.valid() || !destination.valid()) return false;

    const float rect[4] = { sourceRect.x, sourceRect.y, sourceRect.z, sourceRect.w };
    mDriver->updateBuffer(mParams, 0, rect, sizeof(rect));

    prisma::RenderPassDesc pass;
    pass.colors[0].texture = destination;
    pass.colors[0].mip = mip;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::Clear;
    pass.clearColor[0] = 0.0f;
    pass.clearColor[1] = 0.0f;
    pass.clearColor[2] = 0.0f;
    pass.clearColor[3] = 0.0f;
    pass.depthLoad = prisma::LoadOp::DontCare;
    pass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);
    prisma::Viewport area;
    area.width = static_cast<float>(width);
    area.height = static_cast<float>(height);
    mDriver->setViewport(area);
    mDriver->bindPipeline(destinationSrgb ? mPipelineSrgb : mPipeline);
    mDriver->bindUniformBuffer(0, mParams, 0, sizeof(rect));
    mDriver->bindTexture(0, source, mSampler);
    mDriver->draw(3, 0);
    mDriver->endRenderPass();
    return true;
}

prisma::TextureHandle DecalAtlas::getTexture() const
{
    return mTexture;
}

prisma::TextureHandle DecalAtlas::getTextureSrgb() const
{
    return mTextureSrgb;
}

std::uint32_t DecalAtlas::getWidth() const
{
    return mWidth;
}

std::uint32_t DecalAtlas::getHeight() const
{
    return mHeight;
}

int DecalAtlas::getMipmaps() const
{
    return mMipmaps;
}

bool DecalAtlas::isDirty() const
{
    return mDirty;
}

} // namespace prisma3d
