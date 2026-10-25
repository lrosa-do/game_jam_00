#include "prisma3d/scene/Ocean3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PlanarReflector3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const char* const kName = "ocean";
const float kHalfPi = 1.57079632679f;
const float kReflectorHalfSize = 1000.0f;
const std::uint32_t kReadbackKeepAliveFrames = 8;

float fraction(float value)
{
    return value - floorf(value);
}

} // namespace

Ocean3D::Ocean3D()
        : VisualInstance3D(),
          mSimulation(nullptr),
          mParams(),
          mMesh(),
          mMaterial(),
          mWaterColor(0.0f, 2.0f / 255.0f, 6.0f / 255.0f, 0.6f),
          mExtinctionColor(0.0f, 0.9f, 1.0f),
          mSurfaceDetail(4),
          mTolerance(2.0f),
          mWaterHeight(0.0f),
          mPlanar(false),
          mReflectionResolution(512),
          mReflector(nullptr),
          mDirty(true),
          mFields(),
          mFieldsRequest(0),
          mFieldsValid(false)
{
    setNotifyTransform(true);
    setCastShadows(false);
    if (TextureStore* textures = TextureStore::singleton())
        mSimulation = new OceanSimulation(*textures);
    if (MaterialStore* materials = MaterialStore::singleton()) mMaterial = materials->create(kName);
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetMaterialOverride(getInstance(), mMaterial);
    rebuildGrid();
}

Ocean3D::~Ocean3D()
{
    if (RenderingServer* server = RenderingServer::singleton())
    {
        server->framePreDrawDisconnect(this);
        server->instanceSetMaterialOverride(getInstance(), MaterialId());
        server->instanceSetBase(getInstance(), MeshId());
    }
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mMaterial.valid()) materials->destroy(mMaterial);
    delete mSimulation;
}

bool Ocean3D::isSupported() const
{
    TextureStore* textures = TextureStore::singleton();
    return textures && OceanSimulation::supported(textures->driver());
}

void Ocean3D::setSimulationParams(const OceanSimulationParams& params)
{
    mParams = params;
    mDirty = true;
    mFieldsValid = false;
}

const OceanSimulationParams& Ocean3D::getSimulationParams() const
{
    return mParams;
}

void Ocean3D::setWaterColor(const Math::Vec4& colour)
{
    mWaterColor = colour;
    applyMaterial();
}

Math::Vec4 Ocean3D::getWaterColor() const
{
    return mWaterColor;
}

void Ocean3D::setExtinctionColor(const Math::Vec3& colour)
{
    mExtinctionColor = colour;
    applyMaterial();
}

Math::Vec3 Ocean3D::getExtinctionColor() const
{
    return mExtinctionColor;
}

void Ocean3D::setSurfaceDetail(std::uint32_t detail)
{
    if (detail < 1) detail = 1;
    if (detail > 8) detail = 8;
    if (detail == mSurfaceDetail) return;
    mSurfaceDetail = detail;
    rebuildGrid();
    applyMaterial();
}

std::uint32_t Ocean3D::getSurfaceDetail() const
{
    return mSurfaceDetail;
}

void Ocean3D::setDisplacementTolerance(float tolerance)
{
    mTolerance = tolerance;
    applyMaterial();
}

float Ocean3D::getDisplacementTolerance() const
{
    return mTolerance;
}

float Ocean3D::getWaterHeight() const
{
    return mWaterHeight;
}

void Ocean3D::setPlanarReflection(bool enabled)
{
    mPlanar = enabled;
    applyReflection();
}

bool Ocean3D::hasPlanarReflection() const
{
    return mPlanar;
}

void Ocean3D::setReflectionResolution(std::uint32_t resolution)
{
    mReflectionResolution = resolution;
    applyReflection();
}

std::uint32_t Ocean3D::getReflectionResolution() const
{
    return mReflectionResolution;
}

MaterialId Ocean3D::getMaterial() const
{
    return mMaterial;
}

const OceanSimulation* Ocean3D::getSimulation() const
{
    return mSimulation;
}

Math::Vec3 Ocean3D::getDisplacedPosition(const Math::Vec3& worldPosition)
{
    mFieldsRequest = kReadbackKeepAliveFrames;
    Math::Vec3 position(worldPosition.x, mWaterHeight, worldPosition.z);
    const std::size_t size = OceanSimulation::kSize;
    if (!mFieldsValid || mFields.size() != size * size * 4) return position;

    const float patchRecip = 1.0f / mParams.patchLength;
    const float u = fraction(position.x * patchRecip);
    const float v = fraction(position.z * patchRecip);
    const float fx = u * static_cast<float>(size - 1);
    const float fy = v * static_cast<float>(size - 1);
    const std::size_t x0 = static_cast<std::size_t>(floorf(fx));
    const std::size_t y0 = static_cast<std::size_t>(floorf(fy));
    const std::size_t x1 = static_cast<std::size_t>(ceilf(fx));
    const std::size_t y1 = static_cast<std::size_t>(ceilf(fy));
    const float tx = fx - floorf(fx);
    const float ty = fy - floorf(fy);
    const std::size_t second = size * size * 2;
    const auto texel = [&](std::size_t x, std::size_t y) -> Math::Vec3
    {
        const std::size_t index = y * size + x;
        const float sign = ((x + y) & 1u) ? -1.0f : 1.0f;
        return Math::Vec3(mFields[index * 2 + 1] * sign * mParams.choppyScale,
                mFields[second + index * 2] * sign * mParams.choppyScale,
                mFields[index * 2] * sign);
    };
    const Math::Vec3 top = texel(x0, y0) * (1.0f - tx) + texel(x1, y0) * tx;
    const Math::Vec3 bottom = texel(x0, y1) * (1.0f - tx) + texel(x1, y1) * tx;
    const Math::Vec3 displacement = top * (1.0f - ty) + bottom * ty;
    position.x += displacement.x;
    position.y += displacement.z;
    position.z += displacement.y;
    return position;
}

void Ocean3D::update()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || !mSimulation || !isVisibleInTree()) return;
    if (mDirty)
    {
        mDirty = false;
        if (!mSimulation->create(mParams))
        {
            applyMaterial();
            return;
        }
        applyMaterial();
    }
    if (!mSimulation->valid()) return;
    server->openFrame();
    mSimulation->update(server->time());
    if (mFieldsRequest > 0)
    {
        --mFieldsRequest;
        mFieldsValid = mSimulation->readFields(mFields);
    }
}

void Ocean3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            if (RenderingServer* server = RenderingServer::singleton())
                server->framePreDrawConnect(this, [this]() { update(); });
            break;
        case kNotificationExitTree:
            if (RenderingServer* server = RenderingServer::singleton())
                server->framePreDrawDisconnect(this);
            break;
        case kNotificationEnterWorld:
        case kNotificationTransformChanged:
            applyTransform();
            break;
        default:
            break;
    }
}

void Ocean3D::applyTransform()
{
    const float height = getGlobalTransform().GetTranslation().y;
    if (height != mWaterHeight)
    {
        mWaterHeight = height;
        applyMaterial();
    }
}

void Ocean3D::rebuildGrid()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return;
    if (!store->valid(mMesh)) mMesh = store->create(kName);
    else
        store->clearSurfaces(mMesh);
    if (!mMesh.valid()) return;

    const std::uint32_t width = 160u * mSurfaceDetail;
    const std::uint32_t height = 90u * mSurfaceDetail;
    SurfaceData surface;
    surface.positions.reserve(static_cast<std::size_t>(width) * height);
    for (std::uint32_t y = 0; y < height; ++y)
        for (std::uint32_t x = 0; x < width; ++x)
            surface.positions.push_back(
                    Math::Vec3(static_cast<float>(x) / static_cast<float>(width - 1), 0.0f,
                            static_cast<float>(y) / static_cast<float>(height - 1)));
    surface.normals.resize(surface.positions.size(), Math::Vec3(0.0f, 1.0f, 0.0f));
    surface.uvs.resize(surface.positions.size(), Math::Vec2(0.0f, 0.0f));
    surface.indices.reserve(static_cast<std::size_t>(width - 1) * (height - 1) * 6);
    for (std::uint32_t x = 0; x + 1 < width; x++)
    {
        for (std::uint32_t y = 0; y + 1 < height; y++)
        {
            const std::uint32_t lowerLeft = x + y * width;
            const std::uint32_t lowerRight = (x + 1) + y * width;
            const std::uint32_t topLeft = x + (y + 1) * width;
            const std::uint32_t topRight = (x + 1) + (y + 1) * width;

            surface.indices.push_back(topLeft);
            surface.indices.push_back(lowerLeft);
            surface.indices.push_back(lowerRight);

            surface.indices.push_back(topLeft);
            surface.indices.push_back(lowerRight);
            surface.indices.push_back(topRight);
        }
    }
    if (!store->addSurface(mMesh, surface, kName)) return;
    const float huge = 1.0e7f;
    store->setSurfaceAabb(mMesh, 0,
            Math::Box(Math::Vec3(-huge, -huge, -huge), Math::Vec3(huge, huge, huge)));
    setBase(mMesh);
}

void Ocean3D::applyMaterial()
{
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials || !mMaterial.valid()) return;
    OceanMaterialData data;
    data.enabled = mSimulation && mSimulation->valid();
    if (data.enabled)
    {
        data.displacementMap = mSimulation->displacementMap();
        data.gradientMap = mSimulation->gradientMap();
        data.perlinMap = mSimulation->perlinMap();
    }
    data.waterColor = mWaterColor;
    data.extinctionColor = mExtinctionColor;
    data.waterHeight = mWaterHeight;
    data.patchLength = mParams.patchLength;
    data.waveAmplitude = mParams.waveAmplitude;
    data.timeScale = mParams.timeScale;
    data.displacementTolerance = mTolerance;
    data.gridWidth = 160u * mSurfaceDetail;
    data.gridHeight = 90u * mSurfaceDetail;
    materials->setOcean(mMaterial, data);
    materials->setCull(mMaterial, MaterialCull::Disabled);
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetVisible(getInstance(), data.enabled && isVisibleInTree());
}

void Ocean3D::applyReflection()
{
    if (mPlanar)
    {
        if (!mReflector)
        {
            mReflector = createChild<PlanarReflector3D>();
            mReflector->setInternal(true);
            mReflector->rotateX(-kHalfPi);
            mReflector->setHalfSize(Math::Vec2(kReflectorHalfSize, kReflectorHalfSize));
            mReflector->setMaxDistance(kReflectorHalfSize);
            setReflector(mReflector);
        }
        mReflector->setResolution(mReflectionResolution);
    }
    else if (mReflector)
    {
        setReflector(nullptr);
        mReflector->setVisible(false);
        mReflector->queueFree();
        mReflector = nullptr;
    }
}

void Ocean3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Ocean3D;
    using S = OceanSimulationParams;
    auto params = &P::getSimulationParams;
    auto setParams = &P::setSimulationParams;
    auto real = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true); };

    DB::addPropertyGroup<P>("Simulation", "simulation_");
    DB::addFieldProperty<P>(real("simulation_patch_length", 1.0, 10000.0, 0.01), params, setParams,
            &S::patchLength);
    DB::addFieldProperty<P>(real("simulation_time_scale", 0.0, 100.0, 0.001), params, setParams,
            &S::timeScale);
    DB::addFieldProperty<P>(real("simulation_wave_amplitude", 0.0, 100000.0, 0.1), params,
            setParams, &S::waveAmplitude);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Vec2, "simulation_wind_direction"), params,
            setParams, &S::windDirection);
    DB::addFieldProperty<P>(real("simulation_wind_speed", 0.0, 100000.0, 0.1), params, setParams,
            &S::windSpeed);
    DB::addFieldProperty<P>(real("simulation_wind_dependency", 0.0, 1.0, 0.001), params, setParams,
            &S::windDependency);
    DB::addFieldProperty<P>(real("simulation_choppy_scale", 0.0, 100.0, 0.01), params, setParams,
            &S::choppyScale);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Int, "simulation_seed"), params, setParams,
            &S::seed);

    DB::addPropertyGroup<P>("Surface", "");
    DB::addProperty<P>(PropertyInfo::color("water_color"), &P::setWaterColor, &P::getWaterColor);
    DB::addProperty<P>(PropertyInfo::color("extinction_color", true), &P::setExtinctionColor,
            &P::getExtinctionColor);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "surface_detail", 1.0, 8.0, 1.0),
            &P::setSurfaceDetail, &P::getSurfaceDetail);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "displacement_tolerance", 0.0,
                               100.0, 0.01, true),
            &P::setDisplacementTolerance, &P::getDisplacementTolerance);

    DB::addPropertyGroup<P>("Reflection", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "planar_reflection"),
            &P::setPlanarReflection, &P::hasPlanarReflection);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "reflection_resolution", 16.0, 8192.0, 1.0),
            &P::setReflectionResolution, &P::getReflectionResolution);
}

} // namespace prisma3d
