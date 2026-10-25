#include "prisma3d/scene/Water3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/ResourceProperties.h"

namespace prisma3d
{

namespace
{

const char* const kMeshName = "water";
const float kHalfPi = 1.57079632679f;

} // namespace

Water3D::Water3D()
        : VisualInstance3D(),
          mMesh(),
          mMaterial(),
          mNormalTexture(),
          mSize(10.0f, 10.0f),
          mUvTiling(0.01f),
          mColor(0.032f, 0.10f, 0.16f, 0.17f),
          mRoughness(0.2f),
          mSpecular(1.0f),
          mRefraction(0.05f),
          mRefractionEnabled(true),
          mWaves(),
          mReflection(WaterReflection::Sky),
          mReflectionResolution(512),
          mReflectionMaxDistance(8.0f),
          mReflector(nullptr)
{
    if (MaterialStore* materials = MaterialStore::singleton())
        mMaterial = materials->create(kMeshName);
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetMaterialOverride(getInstance(), mMaterial);
    rebuildMesh();
    applyMaterial();
}

Water3D::~Water3D()
{
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetBase(getInstance(), MeshId());
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mMaterial.valid()) materials->destroy(mMaterial);
}

void Water3D::setSize(const Math::Vec2& size)
{
    mSize = size;
    rebuildMesh();
    applyReflection();
}

Math::Vec2 Water3D::getSize() const
{
    return mSize;
}

void Water3D::setUvTiling(float tilesPerMetre)
{
    mUvTiling = tilesPerMetre;
    rebuildMesh();
}

float Water3D::getUvTiling() const
{
    return mUvTiling;
}

void Water3D::setNormalTexture(TextureId texture)
{
    mNormalTexture = texture;
    applyMaterial();
}

TextureId Water3D::getNormalTexture() const
{
    return mNormalTexture;
}

void Water3D::setColor(const Math::Vec4& colour)
{
    mColor = colour;
    applyMaterial();
}

Math::Vec4 Water3D::getColor() const
{
    return mColor;
}

void Water3D::setRoughness(float roughness)
{
    mRoughness = roughness;
    applyMaterial();
}

float Water3D::getRoughness() const
{
    return mRoughness;
}

void Water3D::setSpecular(float specular)
{
    mSpecular = specular;
    applyMaterial();
}

float Water3D::getSpecular() const
{
    return mSpecular;
}

void Water3D::setRefraction(float strength)
{
    mRefraction = strength;
    applyMaterial();
}

float Water3D::getRefraction() const
{
    return mRefraction;
}

void Water3D::setRefractionEnabled(bool enabled)
{
    mRefractionEnabled = enabled;
    applyMaterial();
}

bool Water3D::isRefractionEnabled() const
{
    return mRefractionEnabled;
}

void Water3D::setWaves(const WaterMaterialData& waves)
{
    mWaves = waves;
    applyMaterial();
}

const WaterMaterialData& Water3D::getWaves() const
{
    return mWaves;
}

void Water3D::setWind(const Math::Vec2& direction, float speed)
{
    mWaves.windDirection = direction;
    mWaves.windSpeed = speed;
    applyMaterial();
}

void Water3D::setReflection(WaterReflection mode)
{
    mReflection = mode;
    applyReflection();
}

WaterReflection Water3D::getReflection() const
{
    return mReflection;
}

void Water3D::setReflectionResolution(std::uint32_t resolution)
{
    mReflectionResolution = resolution;
    applyReflection();
}

std::uint32_t Water3D::getReflectionResolution() const
{
    return mReflectionResolution;
}

void Water3D::setReflectionMaxDistance(float distance)
{
    mReflectionMaxDistance = distance;
    applyReflection();
}

float Water3D::getReflectionMaxDistance() const
{
    return mReflectionMaxDistance;
}

PlanarReflector3D* Water3D::getPlanarReflector() const
{
    return mReflector;
}

MaterialId Water3D::getMaterial() const
{
    return mMaterial;
}

MeshId Water3D::getMesh() const
{
    return mMesh;
}

void Water3D::rebuildMesh()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return;
    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        if (!mMesh.valid()) return;
    }
    else
        store->clearSurfaces(mMesh);

    SurfaceData surface;
    planeMeshArrays(surface, mSize.x, mSize.y, 1, 1, 1.0f);
    const Math::Vec2 repeat(mUvTiling * mSize.x, mUvTiling * mSize.y);
    for (std::size_t i = 0; i < surface.uvs.size(); ++i)
    {
        surface.uvs[i].x *= repeat.x;
        surface.uvs[i].y *= repeat.y;
    }
    store->addSurface(mMesh, surface, kMeshName);
    setBase(mMesh);
}

void Water3D::applyMaterial()
{
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials || !mMaterial.valid()) return;
    materials->setColor(mMaterial, MaterialParam::Albedo,
            mRefractionEnabled ? mColor : Math::Vec4(mColor.x, mColor.y, mColor.z, 0.8f));
    materials->setFloat(mMaterial, MaterialParam::Metallic, 0.0f);
    materials->setFloat(mMaterial, MaterialParam::Roughness, mRoughness);
    materials->setFloat(mMaterial, MaterialParam::Specular, mSpecular);
    materials->setFloat(mMaterial, MaterialParam::Refraction,
            mRefractionEnabled ? mRefraction : 0.0f);
    materials->setTransparency(mMaterial,
            mRefractionEnabled ? MaterialTransparency::Opaque : MaterialTransparency::AlphaBlend);
    materials->setTexture(mMaterial, MaterialTexture::Normal, mNormalTexture);
    materials->setWater(mMaterial, true, mWaves);
}

void Water3D::applyReflection()
{
    if (mReflection == WaterReflection::Planar)
    {
        if (!mReflector)
        {
            mReflector = createChild<PlanarReflector3D>();
            mReflector->setInternal(true);
            mReflector->rotateX(-kHalfPi);
            setReflector(mReflector);
        }
        mReflector->setHalfSize(Math::Vec2(mSize.x * 0.5f, mSize.y * 0.5f));
        mReflector->setResolution(mReflectionResolution);
        mReflector->setMaxDistance(mReflectionMaxDistance);
    }
    else if (mReflector)
    {
        setReflector(nullptr);
        mReflector->setVisible(false);
        mReflector->queueFree();
        mReflector = nullptr;
    }
}

void Water3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Water3D;
    using W = WaterMaterialData;
    auto waves = &P::getWaves;
    auto setWaves = &P::setWaves;
    auto real = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true, true); };

    DB::addProperty<P>(PropertyInfo::range(PropertyType::Vec2, "size", 0.001, 100000.0, 0.01, true),
            &P::setSize, &P::getSize);
    DB::addProperty<P>(real("uv_tiling", 0.0, 100.0, 0.0001), &P::setUvTiling, &P::getUvTiling);
    resourceprop::add<P, TextureId>(PropertyInfo::resource("normal_texture", "Texture2D"),
            &P::setNormalTexture, &P::getNormalTexture, false);
    DB::addProperty<P>(PropertyInfo::color("color"), &P::setColor, &P::getColor);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "roughness", 0.0, 1.0, 0.01),
            &P::setRoughness, &P::getRoughness);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "specular", 0.0, 1.0, 0.01, true),
            &P::setSpecular, &P::getSpecular);

    DB::addPropertyGroup<P>("Refraction", "refraction_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "refraction_enabled"),
            &P::setRefractionEnabled, &P::isRefractionEnabled);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "refraction_strength", 0.0, 1.0, 0.001, true),
            &P::setRefraction, &P::getRefraction);

    DB::addPropertyGroup<P>("Waves", "waves_");
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec2, "waves_wind_direction"), waves,
            setWaves, &W::windDirection);
    DB::addFieldProperty<P>(real("waves_wind_speed", 0.0, 100.0, 0.01), waves, setWaves,
            &W::windSpeed);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec2, "waves_big"), waves, setWaves,
            &W::bigWaves);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec2, "waves_mid"), waves, setWaves,
            &W::midWaves);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec2, "waves_small"), waves, setWaves,
            &W::smallWaves);
    DB::addFieldProperty<P>(real("waves_scale", 0.0, 1000.0, 0.01), waves, setWaves, &W::waveScale);
    DB::addFieldProperty<P>(real("waves_chop", 0.0, 10.0, 0.001), waves, setWaves, &W::waveChop);
    DB::addFieldProperty<P>(real("waves_bump", 0.0, 100.0, 0.01), waves, setWaves, &W::bump);
    DB::addFieldProperty<P>(real("waves_bump2", 0.0, 100.0, 0.01), waves, setWaves, &W::bump2);
    DB::addFieldProperty<P>(real("waves_specular_mix", 0.0, 1.0, 0.001), waves, setWaves,
            &W::specularMix);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec3, "waves_depth_fade"), waves, setWaves,
            &W::depthFade);
    DB::addFieldProperty<P>(real("waves_depth_scale", 0.0, 100.0, 0.001), waves, setWaves,
            &W::depthScale);
    DB::addFieldProperty<P>(real("waves_shore_fade", 0.0, 1000.0, 0.01), waves, setWaves,
            &W::shoreFade);

    DB::addPropertyGroup<P>("Reflection", "reflection_");
    DB::addProperty<P>(PropertyInfo::enumeration("reflection_mode", "Sky,Planar"),
            &P::setReflection, &P::getReflection);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "reflection_resolution", 16.0, 8192.0, 1.0),
            &P::setReflectionResolution, &P::getReflectionResolution);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "reflection_max_distance", 0.0,
                               4096.0, 0.01, true),
            &P::setReflectionMaxDistance, &P::getReflectionMaxDistance);
}

} // namespace prisma3d
