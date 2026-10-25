#include "prisma3d/scene/LightmapGI.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "dds.h"
#include "platform.h"

#include <ct/json.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

constexpr std::uint32_t kBytesPerTexel = 8;

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = path.size();
    while (slash > 0 && path[slash - 1] != '/' && path[slash - 1] != '\\') --slash;
    if (slash == 0) return ct::String();
    return ct::String(path.c_str(), slash);
}

ct::String fileNameOf(const ct::String& path)
{
    return path.substr(directoryOf(path).size());
}

ct::String withoutExtension(const ct::String& name)
{
    std::size_t dot = name.size();
    while (dot > 0 && name[dot - 1] != '.') --dot;
    if (dot <= 1) return name;
    return name.substr(0, dot - 1);
}

void setError(ct::String* error, const ct::String& message)
{
    if (error) *error = message;
}

} // namespace

LightmapGIData::LightmapGIData()
        : mLight(),
          mShadowmask(),
          mUsers(),
          mUsesSphericalHarmonics(false),
          mInterior(false),
          mBakedExposure(1.0f),
          mBounds(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f)),
          mProbeOrigin(0.0f, 0.0f, 0.0f),
          mProbeCell(1.0f, 1.0f, 1.0f),
          mProbeDims(),
          mProbeSh()
{
}

LightmapGIData::~LightmapGIData()
{
    freeLayers(mLight);
    freeLayers(mShadowmask);
}

void LightmapGIData::freeLayers(Layers& target)
{
    TextureStore* store = TextureStore::singleton();
    if (store && target.texture.valid()) store->destroy(target.texture);
    target = Layers();
}

void LightmapGIData::addUser(const ct::String& path, const Math::Vec4& uvScale, int sliceIndex,
        int subInstance)
{
    User user;
    user.path = path;
    user.uvScale = uvScale;
    user.sliceIndex = sliceIndex;
    user.subInstance = subInstance;
    mUsers.push_back(user);
}

std::size_t LightmapGIData::getUserCount() const
{
    return mUsers.size();
}

const LightmapGIData::User& LightmapGIData::getUser(std::size_t index) const
{
    return mUsers[index];
}

void LightmapGIData::clearUsers()
{
    mUsers.clear();
}

bool LightmapGIData::setImages(Layers& target, std::uint32_t width, std::uint32_t height,
        std::uint32_t layers, const std::uint16_t* rgba16f, const char* name)
{
    TextureStore* store = TextureStore::singleton();
    if (!store || width == 0 || height == 0 || layers == 0 || !rgba16f)
    {
        log_error("lightmap: invalid images");
        return false;
    }
    prisma::TextureDesc desc;
    desc.type = prisma::TextureType::Texture2DArray;
    desc.format = prisma::TextureFormat::RGBA16F;
    desc.width = width;
    desc.height = height;
    desc.depth = layers;
    desc.mipLevels = 1;
    desc.debugName = name;
    const TextureId created = store->create(desc, false);
    if (!created.valid())
    {
        log_error("lightmap: could not create the texture");
        return false;
    }
    const prisma::TextureHandle gpu = store->gpu(created);
    const std::size_t layerTexels = static_cast<std::size_t>(width) * height * 4;
    for (std::uint32_t layer = 0; layer < layers; ++layer)
        store->driver()->updateTexture(gpu, 0, layer, rgba16f + layerTexels * layer);
    freeLayers(target);
    target.texture = created;
    target.width = width;
    target.height = height;
    target.layers = layers;
    target.pixels.resize(layerTexels * layers);
    memcpy(target.pixels.data(), rgba16f, layerTexels * layers * sizeof(std::uint16_t));
    return true;
}

bool LightmapGIData::setLightImages(std::uint32_t width, std::uint32_t height, std::uint32_t layers,
        const std::uint16_t* rgba16f)
{
    return setImages(mLight, width, height, layers, rgba16f, "lightmap");
}

bool LightmapGIData::setShadowmaskImages(std::uint32_t width, std::uint32_t height,
        std::uint32_t layers, const std::uint16_t* rgba16f)
{
    return setImages(mShadowmask, width, height, layers, rgba16f, "lightmap shadowmask");
}

void LightmapGIData::clearShadowmaskTextures()
{
    freeLayers(mShadowmask);
}

bool LightmapGIData::hasShadowmaskTextures() const
{
    return mShadowmask.texture.valid();
}

TextureId LightmapGIData::getLightTexture() const
{
    return mLight.texture;
}

TextureId LightmapGIData::getShadowmaskTexture() const
{
    return mShadowmask.texture;
}

std::uint32_t LightmapGIData::getLayerCount() const
{
    return mLight.layers;
}

std::uint32_t LightmapGIData::getLightWidth() const
{
    return mLight.width;
}

std::uint32_t LightmapGIData::getLightHeight() const
{
    return mLight.height;
}

const ct::Vector<std::uint16_t>& LightmapGIData::getLightPixels() const
{
    return mLight.pixels;
}

void LightmapGIData::clearLightTextures()
{
    freeLayers(mLight);
}

void LightmapGIData::setUsesSphericalHarmonics(bool enable)
{
    mUsesSphericalHarmonics = enable;
}

bool LightmapGIData::isUsingSphericalHarmonics() const
{
    return mUsesSphericalHarmonics;
}

void LightmapGIData::setBounds(const Math::Box& bounds)
{
    mBounds = bounds;
}

const Math::Box& LightmapGIData::getBounds() const
{
    return mBounds;
}

void LightmapGIData::setInterior(bool interior)
{
    mInterior = interior;
}

bool LightmapGIData::isInterior() const
{
    return mInterior;
}

void LightmapGIData::setBakedExposure(float exposure)
{
    mBakedExposure = exposure;
}

float LightmapGIData::getBakedExposure() const
{
    return mBakedExposure;
}

bool LightmapGIData::setProbeGrid(const Math::Vec3& origin, const Math::Vec3& cell,
        const std::uint32_t dims[3], const ct::Vector<float>& sh)
{
    const std::size_t cells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
    if (cells == 0 || sh.size() != cells * 27 || cell.x <= 0.0f || cell.y <= 0.0f || cell.z <= 0.0f)
        return false;
    mProbeOrigin = origin;
    mProbeCell = cell;
    mProbeDims[0] = dims[0];
    mProbeDims[1] = dims[1];
    mProbeDims[2] = dims[2];
    mProbeSh = sh;
    return true;
}

bool LightmapGIData::hasProbes() const
{
    return !mProbeSh.empty();
}

const Math::Vec3& LightmapGIData::getProbeOrigin() const
{
    return mProbeOrigin;
}

const Math::Vec3& LightmapGIData::getProbeCell() const
{
    return mProbeCell;
}

const std::uint32_t* LightmapGIData::getProbeDims() const
{
    return mProbeDims;
}

const ct::Vector<float>& LightmapGIData::getProbeSh() const
{
    return mProbeSh;
}

void LightmapGIData::clear()
{
    freeLayers(mLight);
    freeLayers(mShadowmask);
    mUsers.clear();
    mProbeSh.clear();
    mProbeDims[0] = mProbeDims[1] = mProbeDims[2] = 0;
    mUsesSphericalHarmonics = false;
    mInterior = false;
    mBakedExposure = 1.0f;
    mBounds = Math::Box(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
}

std::uint16_t LightmapGIData::floatToHalf(float value)
{
    std::uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    const std::uint32_t sign = (bits >> 16) & 0x8000u;
    const std::uint32_t exponent = (bits >> 23) & 0xffu;
    std::uint32_t mantissa = bits & 0x7fffffu;
    if (exponent == 0xffu)
        return static_cast<std::uint16_t>(sign | 0x7c00u | (mantissa ? 0x200u : 0u));
    const int half = static_cast<int>(exponent) - 127 + 15;
    if (half >= 0x1f) return static_cast<std::uint16_t>(sign | 0x7c00u);
    if (half <= 0)
    {
        if (half < -10) return static_cast<std::uint16_t>(sign);
        mantissa |= 0x800000u;
        const int shift = 14 - half;
        std::uint32_t result = mantissa >> shift;
        const std::uint32_t remainder = mantissa & ((1u << shift) - 1u);
        const std::uint32_t halfway = 1u << (shift - 1);
        if (remainder > halfway || (remainder == halfway && (result & 1u))) ++result;
        return static_cast<std::uint16_t>(sign | result);
    }
    std::uint32_t result = (static_cast<std::uint32_t>(half) << 10) | (mantissa >> 13);
    const std::uint32_t remainder = mantissa & 0x1fffu;
    if (remainder > 0x1000u || (remainder == 0x1000u && (result & 1u))) ++result;
    return static_cast<std::uint16_t>(sign | result);
}

float LightmapGIData::halfToFloat(std::uint16_t value)
{
    const std::uint32_t sign = (static_cast<std::uint32_t>(value) & 0x8000u) << 16;
    const std::uint32_t exponent = (value >> 10) & 0x1fu;
    const std::uint32_t mantissa = value & 0x3ffu;
    std::uint32_t bits = 0;
    if (exponent == 0)
    {
        if (mantissa == 0) bits = sign;
        else
        {
            std::uint32_t shifted = mantissa;
            int e = -1;
            do
            {
                ++e;
                shifted <<= 1;
            } while ((shifted & 0x400u) == 0);
            bits = sign | (static_cast<std::uint32_t>(127 - 15 - e) << 23) |
                   ((shifted & 0x3ffu) << 13);
        }
    }
    else if (exponent == 0x1fu)
        bits = sign | 0x7f800000u | (mantissa << 13);
    else
        bits = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
    float result = 0.0f;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

bool LightmapGIData::saveLayers(Layers& source, const ct::String& file, ct::String* error)
{
    if (source.pixels.empty())
    {
        if (source.source.empty())
        {
            setError(error, "the texture pixels are no longer in memory");
            return false;
        }
        const ct::Vector<std::uint8_t> original =
                FileSystem::getSingleton().readBinary(source.source);
        if (original.empty() || !FileSystem::writeBinary(file, original.data(), original.size()))
        {
            setError(error, ct::String("could not copy '") + source.source + "' to '" + file + "'");
            return false;
        }
        return true;
    }
    ct::Vector<std::uint8_t> bytes;
    const std::size_t payload = source.pixels.size() * sizeof(std::uint16_t);
    bytes.resize(sizeof(dds::Header) + payload);
    dds::write_header(bytes.data(), dds::DXGI_FORMAT_R16G16B16A16_FLOAT, source.width,
            source.height, 1, source.layers);
    memcpy(bytes.data() + sizeof(dds::Header), source.pixels.data(), payload);
    if (!FileSystem::writeBinary(file, bytes.data(), bytes.size()))
    {
        setError(error, ct::String("could not write '") + file + "'");
        return false;
    }
    return true;
}

bool LightmapGIData::loadLayers(Layers& target, const ct::String& file, std::uint32_t layers,
        const char* name, ct::String* error)
{
    TextureStore* store = TextureStore::singleton();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(file);
    if (bytes.empty())
    {
        setError(error, ct::String("could not read '") + file + "'");
        return false;
    }
    const dds::Header header = dds::read_header(bytes.data(), bytes.size());
    if (!header.is_valid() || header.format() != dds::DXGI_FORMAT_R16G16B16A16_FLOAT ||
            header.is_cubemap() || header.is_3d() || header.mip_levels() != 1 ||
            header.array_size() != layers)
    {
        ct::String message = ct::String("'") + file + "' is not an RGBA16F array with ";
        message.append_number(static_cast<unsigned long long>(layers));
        message += " layers";
        setError(error, message);
        return false;
    }
    const std::size_t layerBytes =
            static_cast<std::size_t>(header.width()) * header.height() * kBytesPerTexel;
    if (header.mip_offset(0, 0) + layerBytes * layers > bytes.size())
    {
        setError(error, ct::String("'") + file + "' is truncated");
        return false;
    }
    if (!store)
    {
        setError(error, "there is no texture store");
        return false;
    }
    prisma::TextureDesc desc;
    desc.type = prisma::TextureType::Texture2DArray;
    desc.format = prisma::TextureFormat::RGBA16F;
    desc.width = header.width();
    desc.height = header.height();
    desc.depth = layers;
    desc.mipLevels = 1;
    desc.debugName = name;
    const TextureId created = store->create(desc, false);
    if (!created.valid())
    {
        setError(error, "the driver could not create the texture");
        return false;
    }
    const prisma::TextureHandle gpu = store->gpu(created);
    for (std::uint32_t layer = 0; layer < layers; ++layer)
        store->driver()->updateTexture(gpu, 0, layer, bytes.data() + header.mip_offset(0, layer));
    freeLayers(target);
    target.texture = created;
    target.width = header.width();
    target.height = header.height();
    target.layers = layers;
    target.source = file;
    return true;
}

bool LightmapGIData::save(const ct::String& path, ct::String* error)
{
    const ct::String directory = directoryOf(path);
    const ct::String base = withoutExtension(fileNameOf(path));

    ct::Json document = ct::Json::object();
    document["format"] = ct::Json(kFormatName);
    document["version"] = ct::Json(static_cast<long long>(kFormatVersion));
    document["uses_spherical_harmonics"] = ct::Json(mUsesSphericalHarmonics);
    document["interior"] = ct::Json(mInterior);
    document["baked_exposure"] = scenejson::fromFloat(mBakedExposure);
    ct::Json bounds = ct::Json::object();
    bounds["min"] = scenejson::fromVec3(mBounds.min);
    bounds["max"] = scenejson::fromVec3(mBounds.max);
    document["bounds"] = bounds;

    if (!mLight.texture.valid())
    {
        setError(error, "there is no light texture to save");
        return false;
    }
    ct::Json light = ct::Json::object();
    const ct::String lightName = base + ".light.dds";
    light["file"] = ct::Json(lightName);
    light["layers"] = ct::Json(static_cast<long long>(mLight.layers));
    document["light_texture"] = light;
    if (!saveLayers(mLight, directory + lightName, error)) return false;

    if (mShadowmask.texture.valid())
    {
        ct::Json shadowmask = ct::Json::object();
        const ct::String shadowName = base + ".shadowmask.dds";
        shadowmask["file"] = ct::Json(shadowName);
        shadowmask["layers"] = ct::Json(static_cast<long long>(mShadowmask.layers));
        document["shadowmask_texture"] = shadowmask;
        if (!saveLayers(mShadowmask, directory + shadowName, error)) return false;
    }

    ct::Json users = ct::Json::array();
    for (std::size_t i = 0; i < mUsers.size(); ++i)
    {
        ct::Json user = ct::Json::object();
        user["path"] = ct::Json(mUsers[i].path);
        user["sub_instance"] = ct::Json(static_cast<long long>(mUsers[i].subInstance));
        user["uv_scale"] = scenejson::fromVec4(mUsers[i].uvScale);
        user["slice"] = ct::Json(static_cast<long long>(mUsers[i].sliceIndex));
        users.push_back(user);
    }
    document["users"] = users;

    const ct::String text = document.dump(2);
    if (!FileSystem::writeText(path, text))
    {
        setError(error, ct::String("could not write '") + path + "'");
        return false;
    }
    return true;
}

void LightmapGIData::releaseImages()
{
    mLight.pixels.clear();
    mShadowmask.pixels.clear();
}

bool LightmapGIData::load(const ct::String& path, ct::String* error)
{
    const ct::String text = FileSystem::getSingleton().readText(path);
    if (text.empty())
    {
        setError(error, ct::String("could not read '") + path + "'");
        return false;
    }
    ct::Json::Error parseError;
    const ct::Json document = ct::Json::parse(text, &parseError);
    if (parseError || !document.is_object())
    {
        setError(error, ct::String("'") + path + "' is not a valid lightmap data file");
        return false;
    }
    const ct::Json* format = document.find("format");
    if (!format || !format->is_string() || format->str() != kFormatName)
    {
        setError(error, ct::String("'") + path + "' is not a lightmap data file");
        return false;
    }
    if (scenejson::getInt(document, "version", 1) > static_cast<std::int64_t>(kFormatVersion))
    {
        setError(error, ct::String("'") + path + "' is newer than this build");
        return false;
    }

    const ct::Json* lightField = document.find("light_texture");
    if (!lightField || !lightField->is_object())
    {
        setError(error, ct::String("'") + path + "' has no light_texture");
        return false;
    }
    const ct::String directory = directoryOf(path);
    const std::uint32_t lightLayers =
            static_cast<std::uint32_t>(scenejson::getInt(*lightField, "layers", 0));
    const ct::String lightFile = scenejson::getString(*lightField, "file", ct::String());

    Layers light;
    if (lightFile.empty() || lightLayers == 0 ||
            !loadLayers(light, directory + lightFile, lightLayers, "lightmap", error))
    {
        if (error && error->empty())
            *error = ct::String("'") + path + "' has no usable light_texture";
        freeLayers(light);
        return false;
    }
    Layers shadowmask;
    if (const ct::Json* shadowField = document.find("shadowmask_texture"))
    {
        if (shadowField->is_object())
        {
            const std::uint32_t layers =
                    static_cast<std::uint32_t>(scenejson::getInt(*shadowField, "layers", 0));
            const ct::String file = scenejson::getString(*shadowField, "file", ct::String());
            if (file.empty() || layers == 0 ||
                    !loadLayers(shadowmask, directory + file, layers, "lightmap shadowmask", error))
            {
                freeLayers(light);
                freeLayers(shadowmask);
                return false;
            }
        }
    }

    clear();
    mLight = light;
    mShadowmask = shadowmask;
    mUsesSphericalHarmonics = scenejson::getBool(document, "uses_spherical_harmonics", false);
    mInterior = scenejson::getBool(document, "interior", false);
    mBakedExposure = scenejson::getFloat(document, "baked_exposure", 1.0f);
    if (const ct::Json* bounds = document.find("bounds"))
    {
        if (bounds->is_object())
        {
            Math::Vec3 minimum(0.0f, 0.0f, 0.0f);
            Math::Vec3 maximum(0.0f, 0.0f, 0.0f);
            scenejson::getVec3(*bounds, "min", minimum);
            scenejson::getVec3(*bounds, "max", maximum);
            mBounds = Math::Box(minimum, maximum);
        }
    }
    if (const ct::Json* users = document.find("users"))
    {
        if (users->is_array())
        {
            for (std::size_t i = 0; i < users->size(); ++i)
            {
                const ct::Json& entry = users->at(i);
                if (!entry.is_object()) continue;
                Math::Vec4 uvScale(0.0f, 0.0f, 1.0f, 1.0f);
                scenejson::getVec4(entry, "uv_scale", uvScale);
                addUser(scenejson::getString(entry, "path", ct::String()), uvScale,
                        static_cast<int>(scenejson::getInt(entry, "slice", 0)),
                        static_cast<int>(scenejson::getInt(entry, "sub_instance", -1)));
            }
        }
    }
    return true;
}

LightmapGI::LightmapGI()
        : Node3D(),
          mLightmap(),
          mData(),
          mDataPath(),
          mVisibleInWorld(false),
          mAssigned(false),
          mBakeQuality(BakeQuality::Medium),
          mUseDenoiser(true),
          mDenoiserStrength(0.1f),
          mDenoiserRange(10),
          mBounces(3),
          mBounceIndirectEnergy(1.0f),
          mBias(0.0005f),
          mTexelScale(1.0f),
          mMaxTextureSize(16384),
          mSupersamplingEnabled(false),
          mSupersamplingFactor(2.0f),
          mInterior(false),
          mEnvironmentMode(EnvironmentMode::Scene),
          mEnvironmentCustomColor(1.0f, 1.0f, 1.0f),
          mEnvironmentCustomEnergy(1.0f),
          mDirectional(false),
          mUseTextureForBounces(true),
          mShadowmaskMode(ShadowmaskMode::None),
          mGenerateProbes(GenerateProbes::Subdiv8),
          mSpecularIntensity(0.0f),
          mModelAtlas(false),
          mMerged(false),
          mCursor()
{
    setNotifyTransform(true);
    RenderingServer* server = RenderingServer::singleton();
    if (server) mLightmap = server->lightmapCreate();
}

LightmapGI::~LightmapGI()
{
    ct::Vector<LightmapGI*>& list = atlases();
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i] == this)
        {
            list.erase(list.begin() + static_cast<std::ptrdiff_t>(i));
            break;
        }
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightmapFree(mLightmap);
}

ct::Vector<LightmapGI*>& LightmapGI::atlases()
{
    static ct::Vector<LightmapGI*> list;
    return list;
}

void LightmapGI::placeBlock(AtlasCursor& cursor, int width, int height, int pageWidth,
        int pageHeight, int gutter, int& x, int& y)
{
    if (cursor.layer < 0)
    {
        cursor.layer = 0;
        cursor.shelfX = 0;
        cursor.shelfY = 0;
        cursor.shelfHeight = height;
    }
    else if (cursor.shelfX + (cursor.shelfX > 0 ? gutter : 0) + width > pageWidth)
    {
        cursor.shelfY += cursor.shelfHeight + gutter;
        cursor.shelfX = 0;
        cursor.shelfHeight = height;
        if (cursor.shelfY + height > pageHeight)
        {
            ++cursor.layer;
            cursor.shelfY = 0;
        }
    }
    x = cursor.shelfX + (cursor.shelfX > 0 ? gutter : 0);
    y = cursor.shelfY;
    cursor.shelfX = x + width;
    if (height > cursor.shelfHeight) cursor.shelfHeight = height;
}

void LightmapGI::setModelAtlas(const AtlasCursor& cursor)
{
    mModelAtlas = true;
    mCursor = cursor;
}

bool LightmapGI::isModelAtlas() const
{
    return mModelAtlas;
}

namespace
{

const Node* topLevelScene(const Node* node)
{
    const Node* current = node;
    while (current->getParent() && current->getParent()->getParent())
        current = current->getParent();
    return current;
}

} // namespace

bool LightmapGI::mergeIntoSharedAtlas()
{
    if (!mData || !mModelAtlas || mMerged) return false;
    ct::Vector<LightmapGI*>& list = atlases();
    LightmapGI* best = nullptr;
    const Node* scene = topLevelScene(this);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        LightmapGI* atlas = list[i];
        if (atlas == this || !atlas->mData || atlas->getTree() != getTree() ||
                topLevelScene(atlas) != scene)
            continue;
        if (atlas->mData->getLightWidth() < mData->getLightWidth() ||
                atlas->mData->getLightHeight() < mData->getLightHeight())
            continue;
        if (!best || atlas->mData->getLightWidth() > best->mData->getLightWidth()) best = atlas;
    }
    return best != nullptr && mergeInto(*best);
}

bool LightmapGI::mergeInto(LightmapGI& atlas)
{
    RenderingServer* server = RenderingServer::singleton();
    LightmapGIData& target = *atlas.mData;
    const LightmapGIData& source = *mData;
    const int pageWidth = static_cast<int>(target.getLightWidth());
    const int pageHeight = static_cast<int>(target.getLightHeight());
    const int width = static_cast<int>(source.getLightWidth());
    const int height = static_cast<int>(source.getLightHeight());
    const std::uint32_t blocks = source.getLayerCount();
    const ct::Vector<std::uint16_t>& pixels = source.getLightPixels();
    if (!server || blocks == 0 || pixels.empty() || target.getLightPixels().empty()) return false;

    const int gutter = 2;
    ct::Vector<int> blockX;
    ct::Vector<int> blockY;
    ct::Vector<int> blockLayer;
    AtlasCursor cursor = atlas.mCursor;
    for (std::uint32_t b = 0; b < blocks; ++b)
    {
        int x = 0;
        int y = 0;
        placeBlock(cursor, width, height, pageWidth, pageHeight, gutter, x, y);
        blockX.push_back(x);
        blockY.push_back(y);
        blockLayer.push_back(cursor.layer);
    }
    std::uint32_t layers = target.getLayerCount();
    if (static_cast<std::uint32_t>(cursor.layer + 1) > layers)
        layers = static_cast<std::uint32_t>(cursor.layer + 1);
    const std::size_t layerTexels = static_cast<std::size_t>(pageWidth) * pageHeight * 4;
    const std::size_t sourceLayerTexels = static_cast<std::size_t>(width) * height * 4;
    ct::Vector<std::uint16_t> texels = target.getLightPixels();
    texels.resize(layerTexels * layers, 0);
    for (std::uint32_t b = 0; b < blocks; ++b)
    {
        std::uint16_t* layer =
                texels.data() + layerTexels * static_cast<std::size_t>(blockLayer[b]);
        const std::uint16_t* block = pixels.data() + sourceLayerTexels * b;
        for (int y = -1; y <= height; ++y)
        {
            const int targetY = blockY[b] + y;
            if (targetY < 0 || targetY >= pageHeight) continue;
            const int sourceY = y < 0 ? 0 : (y >= height ? height - 1 : y);
            const std::uint16_t* row = block + static_cast<std::size_t>(sourceY) * width * 4;
            std::uint16_t* out = layer + static_cast<std::size_t>(targetY) * pageWidth * 4;
            for (int x = -1; x <= width; ++x)
            {
                const int targetX = blockX[b] + x;
                if (targetX < 0 || targetX >= pageWidth) continue;
                const int sourceX = x < 0 ? 0 : (x >= width ? width - 1 : x);
                memcpy(out + targetX * 4, row + sourceX * 4, sizeof(std::uint16_t) * 4);
            }
        }
    }
    if (!target.setLightImages(static_cast<std::uint32_t>(pageWidth),
                static_cast<std::uint32_t>(pageHeight), layers, texels.data()))
        return false;
    atlas.mCursor = cursor;
    atlas.push();

    const float scaleX = static_cast<float>(width) / static_cast<float>(pageWidth);
    const float scaleY = static_cast<float>(height) / static_cast<float>(pageHeight);
    std::size_t assigned = 0;
    for (std::size_t i = 0; i < source.getUserCount(); ++i)
    {
        const LightmapGIData::User& user = source.getUser(i);
        if (user.subInstance >= 0 || user.sliceIndex < 0 ||
                static_cast<std::uint32_t>(user.sliceIndex) >= blocks)
            continue;
        VisualInstance3D* instance = resolveUser(i);
        if (!instance) continue;
        const std::size_t b = static_cast<std::size_t>(user.sliceIndex);
        const Math::Vec4 uvScale(static_cast<float>(blockX[b]) / static_cast<float>(pageWidth) +
                                         user.uvScale.x * scaleX,
                static_cast<float>(blockY[b]) / static_cast<float>(pageHeight) +
                        user.uvScale.y * scaleY,
                user.uvScale.z * scaleX, user.uvScale.w * scaleY);
        server->instanceGeometrySetLightmap(instance->getInstance(), atlas.mLightmap, uvScale,
                blockLayer[b]);
        ++assigned;
    }
    mMerged = true;
    mData->clearLightTextures();
    push();
    log_info("lightmap: %s joined the atlas of %s (%u block(s) of %dx%d, %zu instance(s), atlas "
             "now %u layer(s))",
            getName().c_str(), atlas.getName().c_str(), blocks, width, height, assigned, layers);
    return true;
}

LightmapId LightmapGI::getLightmap() const
{
    return mLightmap;
}

void LightmapGI::push()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->lightmapSetTextures(mLightmap, mData ? mData->getLightTexture() : TextureId(),
            mData && mData->isUsingSphericalHarmonics());
    server->lightmapSetShadowmaskTextures(mLightmap,
            mData ? mData->getShadowmaskTexture() : TextureId());
    server->lightmapSetShadowmaskMode(mLightmap, mShadowmaskMode);
    server->lightmapSetSpecularIntensity(mLightmap, mSpecularIntensity);
    server->lightmapSetBakedExposureNormalization(mLightmap,
            mData ? mData->getBakedExposure() : 1.0f);
    server->lightmapSetBounds(mLightmap,
            mData ? mData->getBounds()
                  : Math::Box(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f)));
    const std::uint32_t none[3] = { 0, 0, 0 };
    if (mData && mData->hasProbes())
        server->lightmapSetProbeGrid(mLightmap, mData->getProbeOrigin(), mData->getProbeCell(),
                mData->getProbeDims(), mData->getProbeSh().data());
    else
        server->lightmapSetProbeGrid(mLightmap, Math::Vec3(0.0f, 0.0f, 0.0f),
                Math::Vec3(1.0f, 1.0f, 1.0f), none, nullptr);
}

void LightmapGI::setLightData(ct::Unique<LightmapGIData> data, const ct::String& path)
{
    if (mData && isInsideTree()) clearLightmaps();
    mData = static_cast<ct::Unique<LightmapGIData>&&>(data);
    mDataPath = path;
    push();
    if (mData && isInsideTree()) assignLightmaps();
}

LightmapGIData* LightmapGI::getLightData() const
{
    return mData.get();
}

void LightmapGI::updateLightData()
{
    push();
}

void LightmapGI::setLightDataPath(const ct::String& path)
{
    if (path.empty())
    {
        setLightData(ct::Unique<LightmapGIData>(), ct::String());
        return;
    }
    ct::Unique<LightmapGIData> data = ct::make_unique<LightmapGIData>();
    ct::String error;
    if (!data->load(path, &error))
    {
        log_warn("lightmap: %s", error.c_str());
        mDataPath = path;
        return;
    }
    setLightData(static_cast<ct::Unique<LightmapGIData>&&>(data), path);
}

const ct::String& LightmapGI::getLightDataPath() const
{
    return mDataPath;
}

VisualInstance3D* LightmapGI::resolveUser(std::size_t index) const
{
    if (!mData) return nullptr;
    Node* node = getNode(mData->getUser(index).path);
    if (!node) return nullptr;
    return node->castTo<VisualInstance3D>();
}

void LightmapGI::assignLightmaps()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!mData || !server) return;
    ct::Vector<ct::String> missing;
    for (std::size_t i = 0; i < mData->getUserCount(); ++i)
    {
        const LightmapGIData::User& user = mData->getUser(i);
        if (user.subInstance >= 0) continue;
        VisualInstance3D* instance = resolveUser(i);
        if (!instance)
        {
            missing.push_back(user.path);
            continue;
        }
        server->instanceGeometrySetLightmap(instance->getInstance(), mLightmap, user.uvScale,
                user.sliceIndex);
    }
    mAssigned = true;
    if (!missing.empty())
    {
        ct::String names;
        for (std::size_t i = 0; i < missing.size() && i < 3; ++i)
        {
            if (i > 0) names += ", ";
            names += missing[i];
        }
        if (missing.size() > 3)
        {
            names += " and ";
            names.append_number(static_cast<unsigned long long>(missing.size() - 3));
            names += " more";
        }
        log_warn("%s couldn't find previously baked nodes and needs a rebake (missing nodes: %s)",
                getName().c_str(), names.c_str());
    }
}

void LightmapGI::clearLightmaps()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!mData || !server) return;
    for (std::size_t i = 0; i < mData->getUserCount(); ++i)
    {
        if (mData->getUser(i).subInstance >= 0) continue;
        VisualInstance3D* instance = resolveUser(i);
        if (!instance) continue;
        server->instanceGeometrySetLightmap(instance->getInstance(), LightmapId(),
                Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f), 0);
    }
    mAssigned = false;
}

void LightmapGI::setBakeQuality(BakeQuality quality)
{
    mBakeQuality = quality;
}

LightmapGI::BakeQuality LightmapGI::getBakeQuality() const
{
    return mBakeQuality;
}

void LightmapGI::setUseDenoiser(bool enable)
{
    mUseDenoiser = enable;
    notifyPropertyListChanged();
}

bool LightmapGI::isUsingDenoiser() const
{
    return mUseDenoiser;
}

void LightmapGI::setDenoiserStrength(float strength)
{
    mDenoiserStrength = strength;
}

float LightmapGI::getDenoiserStrength() const
{
    return mDenoiserStrength;
}

void LightmapGI::setDenoiserRange(int range)
{
    mDenoiserRange = range;
}

int LightmapGI::getDenoiserRange() const
{
    return mDenoiserRange;
}

void LightmapGI::setDirectional(bool enable)
{
    mDirectional = enable;
    notifyPropertyListChanged();
}

bool LightmapGI::isDirectional() const
{
    return mDirectional;
}

void LightmapGI::setShadowmaskMode(ShadowmaskMode mode)
{
    mShadowmaskMode = mode;
    push();
}

ShadowmaskMode LightmapGI::getShadowmaskMode() const
{
    return mShadowmaskMode;
}

void LightmapGI::setUseTextureForBounces(bool enable)
{
    mUseTextureForBounces = enable;
}

bool LightmapGI::isUsingTextureForBounces() const
{
    return mUseTextureForBounces;
}

void LightmapGI::setInterior(bool interior)
{
    mInterior = interior;
}

bool LightmapGI::isInterior() const
{
    return mInterior;
}

void LightmapGI::setEnvironmentMode(EnvironmentMode mode)
{
    mEnvironmentMode = mode;
    notifyPropertyListChanged();
}

LightmapGI::EnvironmentMode LightmapGI::getEnvironmentMode() const
{
    return mEnvironmentMode;
}

void LightmapGI::setEnvironmentCustomColor(const Math::Vec3& color)
{
    mEnvironmentCustomColor = color;
}

const Math::Vec3& LightmapGI::getEnvironmentCustomColor() const
{
    return mEnvironmentCustomColor;
}

void LightmapGI::setEnvironmentCustomEnergy(float energy)
{
    mEnvironmentCustomEnergy = energy;
}

float LightmapGI::getEnvironmentCustomEnergy() const
{
    return mEnvironmentCustomEnergy;
}

void LightmapGI::setBounces(int bounces)
{
    mBounces = bounces < 0 ? 0 : bounces;
}

int LightmapGI::getBounces() const
{
    return mBounces;
}

void LightmapGI::setBounceIndirectEnergy(float energy)
{
    mBounceIndirectEnergy = energy < 0.0f ? 0.0f : energy;
}

float LightmapGI::getBounceIndirectEnergy() const
{
    return mBounceIndirectEnergy;
}

void LightmapGI::setBias(float bias)
{
    mBias = bias < 0.00001f ? 0.00001f : bias;
}

float LightmapGI::getBias() const
{
    return mBias;
}

void LightmapGI::setTexelScale(float scale)
{
    mTexelScale = scale < 0.01f ? 0.01f : scale;
}

float LightmapGI::getTexelScale() const
{
    return mTexelScale;
}

void LightmapGI::setMaxTextureSize(int size)
{
    mMaxTextureSize = size < 2048 ? 2048 : size;
}

int LightmapGI::getMaxTextureSize() const
{
    return mMaxTextureSize;
}

void LightmapGI::setSupersamplingEnabled(bool enable)
{
    mSupersamplingEnabled = enable;
    notifyPropertyListChanged();
}

bool LightmapGI::isSupersamplingEnabled() const
{
    return mSupersamplingEnabled;
}

void LightmapGI::setSupersamplingFactor(float factor)
{
    mSupersamplingFactor = factor < 1.0f ? 1.0f : factor;
}

float LightmapGI::getSupersamplingFactor() const
{
    return mSupersamplingFactor;
}

void LightmapGI::setGenerateProbes(GenerateProbes probes)
{
    mGenerateProbes = probes;
}

LightmapGI::GenerateProbes LightmapGI::getGenerateProbes() const
{
    return mGenerateProbes;
}

void LightmapGI::setSpecularIntensity(float intensity)
{
    mSpecularIntensity = intensity < 0.0f ? 0.0f : intensity;
    push();
}

float LightmapGI::getSpecularIntensity() const
{
    return mSpecularIntensity;
}

Math::Box LightmapGI::getAabb() const
{
    return Math::Box(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
}

void LightmapGI::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name == "supersampling_factor" && !mSupersamplingEnabled)
        info.usage = PropertyUsage::NoEditor;
    else if (info.name == "environment_custom_color" &&
             mEnvironmentMode != EnvironmentMode::CustomColor)
        info.usage = PropertyUsage::NoEditor;
    else if (info.name == "environment_custom_energy" &&
             mEnvironmentMode != EnvironmentMode::CustomColor)
        info.usage = PropertyUsage::NoEditor;
    else if ((info.name == "denoiser_strength" || info.name == "denoiser_range") && !mUseDenoiser)
        info.usage = PropertyUsage::NoEditor;
    else if (info.name == "specular_intensity" && !mDirectional)
        info.usage = PropertyUsage::NoEditor;
}

void LightmapGI::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    RenderingServer* server = RenderingServer::singleton();

    switch (what)
    {
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->lightmapSetScenario(mLightmap, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationPostEnterTree:
            if (mergeIntoSharedAtlas()) break;
            if (mData) assignLightmaps();
            if (mModelAtlas && !mMerged) atlases().push_back(this);
            break;
        case kNotificationExitTree:
        {
            if (mData) clearLightmaps();
            ct::Vector<LightmapGI*>& list = atlases();
            for (std::size_t i = 0; i < list.size(); ++i)
                if (list[i] == this)
                {
                    list.erase(list.begin() + static_cast<std::ptrdiff_t>(i));
                    break;
                }
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server)
                server->lightmapSetTransform(mLightmap, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->lightmapSetScenario(mLightmap, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void LightmapGI::updateVisibility()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();
    mVisibleInWorld = isVisibleInTree();
    if (!server) return;
    server->lightmapSetVisible(mLightmap, mVisibleInWorld);
    if (mVisibleInWorld) server->lightmapSetTransform(mLightmap, getGlobalTransform());
}

void LightmapGI::bindProperties()
{
    using DB = NodeClassDB;
    using P = LightmapGI;
    DB::addPropertyGroup<P>("Tweaks", "");
    DB::addProperty<P>(PropertyInfo::enumeration("quality", "Low,Medium,High,Ultra"),
            &P::setBakeQuality, &P::getBakeQuality);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "supersampling"),
            &P::setSupersamplingEnabled, &P::isSupersamplingEnabled);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "supersampling_factor", 1.0, 8.0, 1.0),
            &P::setSupersamplingFactor, &P::getSupersamplingFactor);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "bounces", 0.0, 6.0, 1.0, true),
            &P::setBounces, &P::getBounces);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "bounce_indirect_energy", 0.0, 2.0, 0.01),
            &P::setBounceIndirectEnergy, &P::getBounceIndirectEnergy);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "directional"), &P::setDirectional,
            &P::isDirectional);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "specular_intensity", 0.0, 10.0, 0.01),
            &P::setSpecularIntensity, &P::getSpecularIntensity);
    DB::addProperty<P>(PropertyInfo::enumeration("shadowmask_mode", "None,Replace,Overlay"),
            &P::setShadowmaskMode, &P::getShadowmaskMode);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_texture_for_bounces"),
            &P::setUseTextureForBounces, &P::isUsingTextureForBounces);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "interior"), &P::setInterior,
            &P::isInterior);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_denoiser"), &P::setUseDenoiser,
            &P::isUsingDenoiser);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "denoiser_strength", 0.001, 0.2, 0.001, true),
            &P::setDenoiserStrength, &P::getDenoiserStrength);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "denoiser_range", 1.0, 20.0, 1.0),
            &P::setDenoiserRange, &P::getDenoiserRange);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "bias", 0.00001, 0.1, 0.00001, true),
            &P::setBias, &P::getBias);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "texel_scale", 0.01, 100.0, 0.01),
            &P::setTexelScale, &P::getTexelScale);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "max_texture_size", 2048.0, 16384.0, 1.0),
            &P::setMaxTextureSize, &P::getMaxTextureSize);

    DB::addPropertyGroup<P>("Environment", "environment_");
    DB::addProperty<P>(PropertyInfo::enumeration("environment_mode", "Disabled,Scene,Custom Color"),
            &P::setEnvironmentMode, &P::getEnvironmentMode);
    DB::addProperty<P>(PropertyInfo::color("environment_custom_color", true),
            &P::setEnvironmentCustomColor, &P::getEnvironmentCustomColor);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "environment_custom_energy", 0.0, 64.0, 0.01),
            &P::setEnvironmentCustomEnergy, &P::getEnvironmentCustomEnergy);

    DB::addPropertyGroup<P>("Gen Probes", "generate_probes_");
    DB::addProperty<P>(PropertyInfo::enumeration("generate_probes_subdiv", "Disabled,4,8,16,32"),
            &P::setGenerateProbes, &P::getGenerateProbes);

    DB::addPropertyGroup<P>("Data", "");
    DB::addProperty<P>(PropertyInfo::file("light_data", "lmdata"), &P::setLightDataPath,
            &P::getLightDataPath);
}

} // namespace prisma3d
