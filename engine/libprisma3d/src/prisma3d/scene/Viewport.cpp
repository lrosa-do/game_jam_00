#include "prisma3d/scene/Viewport.h"

#include "prisma3d/Device.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

Viewport::Viewport()
        : Node(),
          mViewport(),
          mScenario(),
          mWidth(0),
          mHeight(0),
          mClearColor(0.0f, 0.0f, 0.0f, 1.0f),
          mMsaa3D(ViewportMsaa::Disabled),
          mPositionalShadowAtlasSize(4096),
          mPositionalShadowAtlasSixteenBits(false),
          mPositionalShadowAtlasSubdivision{ 4, 4, 16, 64 },
          mCamera(nullptr),
          mCameras()
{

    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;

    mScenario = server->scenarioCreate();
    mViewport = server->viewportCreate();
    server->viewportSetScenario(mViewport, mScenario);
}

Viewport::~Viewport()
{
    for (std::size_t i = 0; i < mCameras.size(); ++i) mCameras[i]->mViewport = nullptr;
    mCameras.clear();
    mCamera = nullptr;

    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;

    server->viewportFree(mViewport);
    server->scenarioFree(mScenario);
}

ViewportId Viewport::getViewportId() const
{
    return mViewport;
}

ScenarioId Viewport::getScenario() const
{
    return mScenario;
}

void Viewport::setSize(std::uint32_t width, std::uint32_t height)
{
    mWidth = width;
    mHeight = height;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetSize(mViewport, width, height);
}

std::uint32_t Viewport::getWidth() const
{
    return mWidth;
}

std::uint32_t Viewport::getHeight() const
{
    return mHeight;
}

float Viewport::getAspect() const
{
    if (mWidth == 0 || mHeight == 0) return 1.0f;
    const Scaling3DLayout layout = RenderingServer::scaling3DLayout(mWidth, mHeight, mScaling3D,
            mScaling3DWidth, mScaling3DHeight, static_cast<Scaling3DAspect>(mScaling3DAspect));
    if (layout.width == 0 || layout.height == 0) return 1.0f;
    return static_cast<float>(layout.width) / static_cast<float>(layout.height);
}

Math::Vec4 Viewport::get3DRect() const
{
    const Scaling3DLayout layout = RenderingServer::scaling3DLayout(mWidth, mHeight, mScaling3D,
            mScaling3DWidth, mScaling3DHeight, static_cast<Scaling3DAspect>(mScaling3DAspect));
    return Math::Vec4(layout.x, layout.y, layout.rectWidth, layout.rectHeight);
}

Math::Vec2 Viewport::getMousePosition() const
{
    Device* device = Device::singleton();
    if (!device) return Math::Vec2(0.0f, 0.0f);

    Math::Vec2 mouse = device->input().getMousePosition();
    int windowWidth = 0;
    int windowHeight = 0;
    device->windowGetSize(&windowWidth, &windowHeight);
    if (windowWidth > 0 && windowHeight > 0)
    {
        mouse.x = mouse.x * static_cast<float>(mWidth) / static_cast<float>(windowWidth);
        mouse.y = mouse.y * static_cast<float>(mHeight) / static_cast<float>(windowHeight);
    }
    return mouse;
}

void Viewport::setClearColor(const Math::Vec4& color)
{
    mClearColor = color;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetClearColor(mViewport, color);
}

Math::Vec4 Viewport::getClearColor() const
{
    return mClearColor;
}

void Viewport::setMsaa3D(ViewportMsaa msaa)
{
    mMsaa3D = msaa;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetMsaa3D(mViewport, msaa);
}

void Viewport::setScaling3D(float scale)
{
    mScaling3D = scale < 0.25f ? 0.25f : scale > 2.0f ? 2.0f : scale;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetScaling3D(mViewport, mScaling3D);
}

float Viewport::getScaling3D() const
{
    return mScaling3D;
}

void Viewport::setScaling3DSize(std::uint32_t width, std::uint32_t height)
{
    mScaling3DWidth = width > 16384u ? 16384u : width;
    mScaling3DHeight = height > 16384u ? 16384u : height;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetScaling3DSize(mViewport, mScaling3DWidth, mScaling3DHeight);
}

std::uint32_t Viewport::getScaling3DWidth() const
{
    return mScaling3DWidth;
}

std::uint32_t Viewport::getScaling3DHeight() const
{
    return mScaling3DHeight;
}

void Viewport::setScaling3DAspect(int aspect)
{
    mScaling3DAspect = aspect < 0 ? 0 : aspect > 3 ? 3 : aspect;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->viewportSetScaling3DAspect(mViewport,
                static_cast<Scaling3DAspect>(mScaling3DAspect));
}

int Viewport::getScaling3DAspect() const
{
    return mScaling3DAspect;
}

void Viewport::setScaling3DMode(int mode)
{
    mScaling3DMode = mode == 1 ? 1 : 0;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->viewportSetScaling3DMode(mViewport, static_cast<Scaling3DMode>(mScaling3DMode));
}

int Viewport::getScaling3DMode() const
{
    return mScaling3DMode;
}

void Viewport::setFsrSharpness(float sharpness)
{
    mFsrSharpness = sharpness < 0.0f ? 0.0f : sharpness > 2.0f ? 2.0f : sharpness;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetFsrSharpness(mViewport, mFsrSharpness);
}

float Viewport::getFsrSharpness() const
{
    return mFsrSharpness;
}

void Viewport::setScreenSpaceAA(int mode)
{
    mScreenSpaceAA = mode == 1 ? 1 : mode == 2 ? 2 : 0;
    RenderingServer* server = RenderingServer::singleton();
    if (server)
    {
        server->viewportSetFxaa(mViewport, mScreenSpaceAA == 1);
        server->viewportSetSmaa(mViewport, mScreenSpaceAA == 2);
    }
}

int Viewport::getScreenSpaceAA() const
{
    return mScreenSpaceAA;
}

void Viewport::setMeshLodThreshold(float pixels)
{
    mMeshLodThreshold = pixels < 0.0f ? 0.0f : pixels;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetMeshLodThreshold(mViewport, mMeshLodThreshold);
}

float Viewport::getMeshLodThreshold() const
{
    return mMeshLodThreshold;
}

ViewportMsaa Viewport::getMsaa3D() const
{
    return mMsaa3D;
}

std::uint32_t Viewport::getMsaaSamples() const
{
    RenderingServer* server = RenderingServer::singleton();
    return server ? server->viewportGetMsaaSamples(mViewport) : 1u;
}

void Viewport::setPositionalShadowAtlasSize(std::uint32_t size, bool sixteenBits)
{
    mPositionalShadowAtlasSize = size;
    mPositionalShadowAtlasSixteenBits = sixteenBits;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportSetPositionalShadowAtlasSize(mViewport, size, sixteenBits);
}

std::uint32_t Viewport::getPositionalShadowAtlasSize() const
{
    return mPositionalShadowAtlasSize;
}

bool Viewport::getPositionalShadowAtlasSixteenBits() const
{
    return mPositionalShadowAtlasSixteenBits;
}

void Viewport::setPositionalShadowAtlasQuadrantSubdivision(std::uint32_t quadrant,
        std::uint32_t subdivision)
{
    if (quadrant >= 4) return;
    mPositionalShadowAtlasSubdivision[quadrant] = subdivision;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->viewportSetPositionalShadowAtlasQuadrantSubdivision(mViewport, quadrant,
                subdivision);
}

std::uint32_t Viewport::getPositionalShadowAtlasQuadrantSubdivision(std::uint32_t quadrant) const
{
    return quadrant < 4 ? mPositionalShadowAtlasSubdivision[quadrant] : 0;
}

Camera3D* Viewport::getCamera3D() const
{
    return mCamera;
}

Viewport* Viewport::findViewport(const Node* node)
{
    if (!node) return nullptr;

    Node* parent = node->getParent();
    while (parent)
    {
        Viewport* viewport = parent->castTo<Viewport>();
        if (viewport) return viewport;
        parent = parent->getParent();
    }
    return nullptr;
}

void Viewport::notificationHandler(int what)
{
    Node::notificationHandler(what);
}

void Viewport::cameraSetCurrent(Camera3D* camera)
{
    if (mCamera == camera) return;

    mCamera = camera;

    RenderingServer* server = RenderingServer::singleton();
    if (server)
        server->viewportAttachCamera(mViewport, camera ? camera->getCameraId() : CameraId());
}

void Viewport::cameraClearCurrent(Camera3D* camera)
{
    if (mCamera != camera) return;

    cameraSetCurrent(nullptr);

    for (std::size_t i = 0; i < mCameras.size(); ++i)
    {
        Camera3D* other = mCameras[i];
        if (other == camera) continue;
        if (!other->isInsideTree()) continue;
        if (mCamera != nullptr) return;

        other->makeCurrent();
    }
}

void Viewport::cameraRemoved(Camera3D* camera)
{
    for (std::size_t i = 0; i < mCameras.size(); ++i)
    {
        if (mCameras[i] == camera)
        {
            mCameras.erase(mCameras.begin() + i);
            break;
        }
    }

    if (mCamera == camera) cameraSetCurrent(nullptr);
}

Window::Window()
        : Viewport()
{
    setProcess(true);

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->viewportAttachToScreen(getViewportId(), true);
}

Window::~Window()
{
}

void Window::notificationHandler(int what)
{
    Viewport::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
        case kNotificationWmSizeChanged:
        case kNotificationProcess:
            syncSize();
            break;
        default:
            break;
    }
}

void Window::syncSize()
{
    if (getWidth() != 0 && getHeight() != 0) return;

    Device* device = Device::singleton();
    if (!device) return;

    int width = 0;
    int height = 0;
    device->windowGetFramebufferSize(&width, &height);
    if (width <= 0 || height <= 0) return;

    setSize(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
}

void Viewport::bindProperties()
{
    using DB = NodeClassDB;
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Vec2, "size", 0.0, 16384.0, 1.0, true),
            [](Viewport* viewport, const PropertyValue& value)
            {
                const Math::Vec2 size = value.asVec2();
                viewport->setSize(static_cast<std::uint32_t>(size.x < 0.0f ? 0.0f : size.x),
                        static_cast<std::uint32_t>(size.y < 0.0f ? 0.0f : size.y));
            },
            [](const Viewport* viewport)
            {
                return PropertyValue::fromVec2(Math::Vec2(static_cast<float>(viewport->getWidth()),
                        static_cast<float>(viewport->getHeight())));
            });
    DB::addProperty<Viewport>(PropertyInfo::color("clear_color"), &Viewport::setClearColor,
            &Viewport::getClearColor);
    DB::addProperty<Viewport>(PropertyInfo::enumeration("msaa_3d", "Disabled,2x,4x,8x"),
            &Viewport::setMsaa3D, &Viewport::getMsaa3D);
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Float, "scaling_3d_scale", 0.25, 2.0, 0.01),
            &Viewport::setScaling3D, &Viewport::getScaling3D);
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Vec2, "scaling_3d_size", 0.0, 16384.0, 1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                const Math::Vec2 size = value.asVec2();
                viewport->setScaling3DSize(
                        static_cast<std::uint32_t>(size.x < 0.0f ? 0.0f : size.x),
                        static_cast<std::uint32_t>(size.y < 0.0f ? 0.0f : size.y));
            },
            [](const Viewport* viewport)
            {
                return PropertyValue::fromVec2(
                        Math::Vec2(static_cast<float>(viewport->getScaling3DWidth()),
                                static_cast<float>(viewport->getScaling3DHeight())));
            });
    DB::addProperty<Viewport>(
            PropertyInfo::enumeration("scaling_3d_aspect", "Stretch,Keep,Crop,Expand"),
            &Viewport::setScaling3DAspect, &Viewport::getScaling3DAspect);
    DB::addProperty<Viewport>(PropertyInfo::enumeration("scaling_3d_mode", "Bilinear,FSR 1.0"),
            &Viewport::setScaling3DMode, &Viewport::getScaling3DMode);
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Float, "fsr_sharpness", 0.0, 2.0, 0.01),
            &Viewport::setFsrSharpness, &Viewport::getFsrSharpness);
    DB::addProperty<Viewport>(PropertyInfo::enumeration("screen_space_aa", "Disabled,FXAA,SMAA"),
            &Viewport::setScreenSpaceAA, &Viewport::getScreenSpaceAA);
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Float, "mesh_lod_threshold", 0.0, 1024.0, 0.1),
            &Viewport::setMeshLodThreshold, &Viewport::getMeshLodThreshold);

    DB::addPropertyGroup<Viewport>("Positional Shadow Atlas", "positional_shadow_atlas_");
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Int, "positional_shadow_atlas_size", 0.0, 16384.0,
                    1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasSize(static_cast<std::uint32_t>(value.asInt()),
                        viewport->getPositionalShadowAtlasSixteenBits());
            },
            &Viewport::getPositionalShadowAtlasSize);
    DB::addProperty<Viewport>(
            PropertyInfo(PropertyType::Bool, "positional_shadow_atlas_16_bits"),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasSize(viewport->getPositionalShadowAtlasSize(),
                        value.asBool());
            },
            &Viewport::getPositionalShadowAtlasSixteenBits);
    const char* quadrantNames[4] = { "positional_shadow_atlas_quad_0",
        "positional_shadow_atlas_quad_1", "positional_shadow_atlas_quad_2",
        "positional_shadow_atlas_quad_3" };
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Int, quadrantNames[0], 0.0, 1024.0, 1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasQuadrantSubdivision(0,
                        static_cast<std::uint32_t>(value.asInt()));
            },
            [](const Viewport* viewport) {
                return PropertyValue::fromInt(
                        viewport->getPositionalShadowAtlasQuadrantSubdivision(0));
            });
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Int, quadrantNames[1], 0.0, 1024.0, 1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasQuadrantSubdivision(1,
                        static_cast<std::uint32_t>(value.asInt()));
            },
            [](const Viewport* viewport) {
                return PropertyValue::fromInt(
                        viewport->getPositionalShadowAtlasQuadrantSubdivision(1));
            });
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Int, quadrantNames[2], 0.0, 1024.0, 1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasQuadrantSubdivision(2,
                        static_cast<std::uint32_t>(value.asInt()));
            },
            [](const Viewport* viewport) {
                return PropertyValue::fromInt(
                        viewport->getPositionalShadowAtlasQuadrantSubdivision(2));
            });
    DB::addProperty<Viewport>(
            PropertyInfo::range(PropertyType::Int, quadrantNames[3], 0.0, 1024.0, 1.0),
            [](Viewport* viewport, const PropertyValue& value)
            {
                viewport->setPositionalShadowAtlasQuadrantSubdivision(3,
                        static_cast<std::uint32_t>(value.asInt()));
            },
            [](const Viewport* viewport) {
                return PropertyValue::fromInt(
                        viewport->getPositionalShadowAtlasQuadrantSubdivision(3));
            });
}

void Window::bindProperties()
{
}

} // namespace prisma3d
