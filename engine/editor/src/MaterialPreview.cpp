#include "prisma3d/editor/MaterialPreview.h"

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

namespace prisma3d
{

namespace
{

const float kSphereRadius = 1.0f;
const float kCubeSize = 1.45f;
const float kCameraDistance = 3.0f;
const float kCameraFov = 45.0f;
const std::uint32_t kSkyRadianceSize = 64;
const Math::Vec4 kBackground(0.13f, 0.14f, 0.16f, 1.0f);

Math::Mat4 aim(const Math::Vec3& forward, const Math::Vec3& up)
{
    const Math::Quaternion rotation = Math::Quaternion::LookRotation(-forward, up);
    return Math::Mat4(rotation.ToMat3(), Math::Vec3(0.0f, 0.0f, 0.0f));
}

} // namespace

MaterialPreview::MaterialPreview(RenderingServer& server)
        : mServer(server),
          mMaterial(),
          mVersion(0),
          mRenders(0),
          mShape(Shape::Sphere),
          mVisible(false),
          mDirty(true)
{
    mScenario = mServer.scenarioCreate();

    EnvironmentSettings environment;
    environment.background = BackgroundMode::Color;
    environment.backgroundColor = Math::Vec3(kBackground.x, kBackground.y, kBackground.z);
    environment.ambientSource = AmbientSource::Sky;
    environment.reflectionSource = ReflectionSource::Sky;
    environment.sky.radianceSize = kSkyRadianceSize;
    mServer.scenarioSetEnvironment(mScenario, environment);

    mViewport = mServer.viewportCreate();
    mServer.viewportSetSize(mViewport, kSize, kSize);
    mServer.viewportSetScenario(mViewport, mScenario);
    mServer.viewportSetClearColor(mViewport, kBackground);
    mServer.viewportSetSrgbOutput(mViewport, true);
    mServer.viewportSetUpdateMode(mViewport, ViewportUpdateMode::Disabled);

    mCamera = mServer.cameraCreate();
    mServer.cameraSetPerspective(mCamera, kCameraFov, 0.1f, 10.0f);
    mServer.cameraSetTransform(mCamera,
            Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, kCameraDistance)));
    mServer.viewportAttachCamera(mViewport, mCamera);

    mKey = mServer.lightCreate(LightType::Directional);
    mServer.lightSetScenario(mKey, mScenario);
    mServer.lightSetTransform(mKey,
            aim(Math::Vec3(-1.0f, -1.0f, -1.0f).Normalized(), Math::Vec3(0.0f, 1.0f, 0.0f)));

    mFill = mServer.lightCreate(LightType::Directional);
    mServer.lightSetScenario(mFill, mScenario);
    mServer.lightSetColor(mFill, Math::Vec3(0.7f, 0.7f, 0.7f));
    mServer.lightSetTransform(mFill,
            aim(Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f)));

    mMeshes[0] = createSphereMesh(mServer.meshes(), kSphereRadius, 32, 48, "PreviewSphere");
    mMeshes[1] = createBoxMesh(mServer.meshes(), Math::Vec3(kCubeSize, kCubeSize, kCubeSize),
            "PreviewCube");

    mInstance = mServer.instanceCreate(mMeshes[0], mScenario);
    mServer.instanceSetVisible(mInstance, false);
}

MaterialPreview::~MaterialPreview()
{
    mServer.instanceFree(mInstance);
    mServer.lightFree(mKey);
    mServer.lightFree(mFill);
    mServer.viewportFree(mViewport);
    mServer.cameraFree(mCamera);
    mServer.scenarioFree(mScenario);
    for (MeshId& mesh: mMeshes) mServer.meshes().destroy(mesh);
}

void MaterialPreview::setMaterial(MaterialId material)
{
    if (mMaterial == material) return;
    mMaterial = material;
    mDirty = true;
}

MaterialId MaterialPreview::material() const
{
    return mMaterial;
}

void MaterialPreview::setShape(Shape shape)
{
    if (mShape == shape) return;
    mShape = shape;
    mServer.instanceSetBase(mInstance, mMeshes[static_cast<int>(shape)]);
    const Math::Mat4 transform = shape == Shape::Cube
                                         ? Math::Mat4::RotationY(0.6f) * Math::Mat4::RotationX(0.5f)
                                         : Math::Mat4::Identity();
    mServer.instanceSetTransform(mInstance, transform);
    mDirty = true;
}

MaterialPreview::Shape MaterialPreview::shape() const
{
    return mShape;
}

void MaterialPreview::setVisible(bool visible)
{
    mVisible = visible;
}

void MaterialPreview::invalidate()
{
    mDirty = true;
}

bool MaterialPreview::update()
{
    if (!mVisible) return false;
    const MaterialInfo* info = mServer.materials().get(mMaterial);
    const std::uint32_t version = info ? info->version : 0;
    if (!mDirty && version == mVersion) return false;
    mDirty = false;
    mVersion = version;
    mServer.instanceSetMaterialOverride(mInstance, info ? mMaterial : MaterialId());
    mServer.instanceSetVisible(mInstance, info != nullptr);
    mServer.viewportSetUpdateMode(mViewport, ViewportUpdateMode::Once);
    ++mRenders;
    return true;
}

TextureId MaterialPreview::texture() const
{
    return mServer.viewportGetTexture(mViewport);
}

unsigned int MaterialPreview::renderCount() const
{
    return mRenders;
}

bool MaterialPreview::readPixel(int x, int y, unsigned char* rgba) const
{
    prisma::Driver* driver = mServer.driver();
    if (!driver) return false;
    prisma::RenderTarget source;
    source.texture = mServer.textures().gpu(mServer.viewportGetTexture(mViewport));
    prisma::Rect area;
    area.x = x;
    area.y = y;
    area.width = 1;
    area.height = 1;
    return driver->readPixels(source, area, rgba);
}

} // namespace prisma3d
