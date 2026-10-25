#include "prisma3d/scene/Camera3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kEpsilon = 1e-8f;
const float kDegreesToRadians = 0.017453292519943295f;

Math::Vec3 normalizedAxis(const Math::Vec3& axis, const Math::Vec3& fallback)
{
    return axis.LengthSquared() > kEpsilon ? axis.Normalized() : fallback;
}

} // namespace

Camera3D::Camera3D()
        : Node3D(),
          mCamera(),
          mViewport(nullptr),
          mProjection(CameraProjection::Perspective),
          mFov(75.0f),
          mSize(1.0f),
          mNear(0.05f),
          mFar(4000.0f),
          mCullMask(0xffffffffu),
          mAttributes(),
          mAttributesOverride(false),
          mCurrent(false)
{
    setNotifyTransform(true);

    RenderingServer* server = RenderingServer::singleton();
    if (server) mCamera = server->cameraCreate();

    updateCameraMode();
}

Camera3D::~Camera3D()
{
    if (mViewport) mViewport->cameraRemoved(this);
    mViewport = nullptr;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->cameraFree(mCamera);
}

CameraId Camera3D::getCameraId() const
{
    return mCamera;
}

void Camera3D::setPerspective(float fovyDegrees, float zNear, float zFar)
{
    const bool changed = mProjection != CameraProjection::Perspective;
    mProjection = CameraProjection::Perspective;
    mFov = fovyDegrees;
    mNear = zNear;
    mFar = zFar;
    updateCameraMode();
    if (changed) notifyPropertyListChanged();
}

void Camera3D::setOrthogonal(float size, float zNear, float zFar)
{
    const bool changed = mProjection != CameraProjection::Orthogonal;
    mProjection = CameraProjection::Orthogonal;
    mSize = size;
    mNear = zNear;
    mFar = zFar;
    updateCameraMode();
    if (changed) notifyPropertyListChanged();
}

void Camera3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name == "fov")
    {
        if (mProjection != CameraProjection::Perspective) info.usage = PropertyUsage::NoEditor;
    }
    else if (info.name == "size")
    {
        if (mProjection != CameraProjection::Orthogonal) info.usage = PropertyUsage::NoEditor;
    }
    else if (info.name.starts_with("attributes_dof_blur_") && !mAttributesOverride)
        info.usage = PropertyUsage::NoEditor;
}

CameraProjection Camera3D::getProjection() const
{
    return mProjection;
}

void Camera3D::setFov(float fovyDegrees)
{
    mFov = fovyDegrees;
    updateCameraMode();
}

float Camera3D::getFov() const
{
    return mFov;
}

void Camera3D::setSize(float size)
{
    mSize = size;
    updateCameraMode();
}

float Camera3D::getSize() const
{
    return mSize;
}

void Camera3D::setNear(float zNear)
{
    mNear = zNear;
    updateCameraMode();
}

float Camera3D::getNear() const
{
    return mNear;
}

void Camera3D::setFar(float zFar)
{
    mFar = zFar;
    updateCameraMode();
}

float Camera3D::getFar() const
{
    return mFar;
}

void Camera3D::setCullMask(std::uint32_t mask)
{
    mCullMask = mask;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->cameraSetCullMask(mCamera, mask);
}

std::uint32_t Camera3D::getCullMask() const
{
    return mCullMask;
}

void Camera3D::setAttributes(const CameraAttributesSettings& attributes)
{
    mAttributes = attributes;
    updateAttributes();
}

const CameraAttributesSettings& Camera3D::getAttributes() const
{
    return mAttributes;
}

void Camera3D::setAttributesOverride(bool enabled)
{
    if (mAttributesOverride == enabled) return;
    mAttributesOverride = enabled;
    updateAttributes();
    notifyPropertyListChanged();
}

bool Camera3D::isAttributesOverride() const
{
    return mAttributesOverride;
}

void Camera3D::makeCurrent()
{
    mCurrent = true;

    if (!isInsideTree()) return;

    Viewport* viewport = mViewport ? mViewport : Viewport::findViewport(this);
    if (viewport) viewport->cameraSetCurrent(this);
}

void Camera3D::clearCurrent(bool enableNext)
{
    mCurrent = false;

    if (!isInsideTree()) return;

    Viewport* viewport = mViewport ? mViewport : Viewport::findViewport(this);
    if (!viewport || viewport->getCamera3D() != this) return;

    if (enableNext) viewport->cameraClearCurrent(this);
    else
        viewport->cameraSetCurrent(nullptr);
}

void Camera3D::setCurrent(bool current)
{
    if (current) makeCurrent();
    else
        clearCurrent();
}

bool Camera3D::isCurrent() const
{
    if (isInsideTree())
    {
        const Viewport* viewport = mViewport ? mViewport : Viewport::findViewport(this);
        if (viewport) return viewport->getCamera3D() == this;
    }
    return mCurrent;
}

Math::Mat4 Camera3D::getCameraTransform() const
{
    const Math::Mat4 global = getGlobalTransform();
    const Math::Mat3 basis(normalizedAxis(global.col0.xyz(), Math::Vec3::UnitX),
            normalizedAxis(global.col1.xyz(), Math::Vec3::UnitY),
            normalizedAxis(global.col2.xyz(), Math::Vec3::UnitZ));
    return Math::Mat4(basis, global.GetTranslation());
}

Math::Mat4 Camera3D::getCameraProjection() const
{
    const Viewport* viewport = mViewport ? mViewport : Viewport::findViewport(this);
    const float aspect = viewport ? viewport->getAspect() : 1.0f;

    if (mProjection == CameraProjection::Orthogonal)
        return RenderingServer::orthogonal(mSize, aspect, mNear, mFar);
    return RenderingServer::perspective(mFov, aspect, mNear, mFar);
}

Viewport* Camera3D::getViewport() const
{
    return mViewport ? mViewport : Viewport::findViewport(this);
}

// A viewport point in the 0..1 space of the 3D picture (which may fill only part of the
// viewport, or more than it, by the viewport's scaling_3d_aspect).
static Math::Vec2 pictureUv(const Viewport* viewport, const Math::Vec2& point)
{
    const Math::Vec4 rect = viewport->get3DRect();
    if (rect.z <= 0.0f || rect.w <= 0.0f) return Math::Vec2(0.5f, 0.5f);
    return Math::Vec2((point.x - rect.x) / rect.z, (point.y - rect.y) / rect.w);
}

static Math::Vec2 picturePoint(const Viewport* viewport, float u, float v)
{
    const Math::Vec4 rect = viewport->get3DRect();
    return Math::Vec2(rect.x + u * rect.z, rect.y + v * rect.w);
}

Math::Vec3 Camera3D::projectRayNormal(const Math::Vec2& point) const
{
    const Math::Vec3 ray = projectLocalRayNormal(point);
    return getCameraTransform().TransformVector(ray).Normalized();
}

Math::Vec3 Camera3D::projectLocalRayNormal(const Math::Vec2& point) const
{
    const Viewport* viewport = getViewport();
    if (!isInsideTree() || !viewport || viewport->getWidth() == 0 || viewport->getHeight() == 0)
    {
        log_error("camera: not inside a scene with a viewport");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    if (mProjection == CameraProjection::Orthogonal) return Math::Vec3(0.0f, 0.0f, -1.0f);

    const Math::Vec2 uv = pictureUv(viewport, point);
    const float halfY = mNear * tanf(mFov * kDegreesToRadians * 0.5f);
    const float halfX = halfY * viewport->getAspect();
    return Math::Vec3((uv.x * 2.0f - 1.0f) * halfX, ((1.0f - uv.y) * 2.0f - 1.0f) * halfY, -mNear)
            .Normalized();
}

Math::Vec3 Camera3D::projectRayOrigin(const Math::Vec2& point) const
{
    const Viewport* viewport = getViewport();
    if (!isInsideTree() || !viewport || viewport->getWidth() == 0 || viewport->getHeight() == 0)
    {
        log_error("camera: not inside a scene with a viewport");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    if (mProjection != CameraProjection::Orthogonal) return getCameraTransform().GetTranslation();

    const Math::Vec2 uv = pictureUv(viewport, point);
    const float x = uv.x;
    const float y = uv.y;
    const float horizontal = mSize * viewport->getAspect();
    const float vertical = mSize;
    const Math::Vec3 ray(x * horizontal - horizontal / 2.0f,
            (1.0f - y) * vertical - vertical / 2.0f, -mNear);
    return getCameraTransform().TransformPoint(ray);
}

Math::Ray Camera3D::projectRay(const Math::Vec2& point) const
{
    return Math::Ray(projectRayOrigin(point), projectRayNormal(point));
}

Math::Ray Camera3D::projectMouseRay() const
{
    const Viewport* viewport = getViewport();
    return projectRay(viewport ? viewport->getMousePosition() : Math::Vec2(0.0f, 0.0f));
}

Math::Vec3 Camera3D::projectPosition(const Math::Vec2& point, float zDepth) const
{
    const Viewport* viewport = getViewport();
    if (!isInsideTree() || !viewport || viewport->getWidth() == 0 || viewport->getHeight() == 0)
    {
        log_error("camera: not inside a scene with a viewport");
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    }
    if (zDepth == 0.0f && mProjection != CameraProjection::Orthogonal)
        return getGlobalTransform().GetTranslation();

    float halfX = 0.0f;
    float halfY = 0.0f;
    if (mProjection == CameraProjection::Orthogonal)
    {
        halfY = mSize * 0.5f;
        halfX = halfY * viewport->getAspect();
    }
    else
    {
        halfY = zDepth * tanf(mFov * kDegreesToRadians * 0.5f);
        halfX = halfY * viewport->getAspect();
    }

    const Math::Vec2 uv = pictureUv(viewport, point);
    const float x = uv.x * 2.0f - 1.0f;
    const float y = (1.0f - uv.y) * 2.0f - 1.0f;
    return getCameraTransform().TransformPoint(Math::Vec3(x * halfX, y * halfY, -zDepth));
}

Math::Vec2 Camera3D::unprojectPosition(const Math::Vec3& position) const
{
    const Viewport* viewport = getViewport();
    if (!isInsideTree() || !viewport)
    {
        log_error("camera: not inside a scene with a viewport");
        return Math::Vec2(0.0f, 0.0f);
    }

    const Math::Vec3 local = getCameraTransform().InverseAffine().TransformPoint(position);
    const Math::Vec4 clip = getCameraProjection() * Math::Vec4(local.x, local.y, local.z, 1.0f);
    if (clip.w > -kEpsilon && clip.w < kEpsilon) return picturePoint(viewport, 0.5f, 0.5f);

    const float x = clip.x / clip.w;
    const float y = clip.y / clip.w;
    return picturePoint(viewport, x * 0.5f + 0.5f, -y * 0.5f + 0.5f);
}

bool Camera3D::pickDepth(const Math::Vec2& point, Math::Vec3& position) const
{
    const Viewport* viewport = getViewport();
    RenderingServer* server = RenderingServer::singleton();
    if (!isInsideTree() || !viewport || !server) return false;

    float depth = 1.0f;
    if (!server->viewportReadDepth(viewport->getViewportId(), static_cast<int>(point.x),
                static_cast<int>(point.y), depth))
        return false;
    if (depth >= 1.0f) return false;

    const float ndc = depth * 2.0f - 1.0f;
    float distance = 0.0f;
    if (mProjection == CameraProjection::Orthogonal)
        distance = (ndc * (mFar - mNear) + mFar + mNear) * 0.5f;
    else
        distance = (2.0f * mFar * mNear) / ((mFar + mNear) - ndc * (mFar - mNear));

    position = projectPosition(point, distance);
    return true;
}

bool Camera3D::isPositionBehind(const Math::Vec3& position) const
{
    const Math::Mat4 transform = getGlobalTransform();
    const Math::Vec3 eye = (transform.col2.xyz() * -1.0f).Normalized();
    return eye.Dot(position - transform.GetTranslation()) < mNear;
}

bool Camera3D::isPositionInFrustum(const Math::Vec3& position) const
{
    const Viewport* viewport = getViewport();
    const Math::Vec3 local = getCameraTransform().InverseAffine().TransformPoint(position);
    const float depth = -local.z;
    if (depth < mNear || depth > mFar) return false;

    float halfY = mSize * 0.5f;
    if (mProjection != CameraProjection::Orthogonal)
        halfY = depth * tanf(mFov * kDegreesToRadians * 0.5f);
    const float halfX = halfY * (viewport ? viewport->getAspect() : 1.0f);
    return local.x >= -halfX && local.x <= halfX && local.y >= -halfY && local.y <= halfY;
}

void Camera3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterWorld:
        {
            mViewport = Viewport::findViewport(this);
            if (!mViewport) break;

            mViewport->mCameras.push_back(this);
            const bool firstCamera = mViewport->mCameras.size() == 1;
            if (mCurrent || firstCamera) mViewport->cameraSetCurrent(this);
            break;
        }
        case kNotificationTransformChanged:
            updateCamera();
            break;
        case kNotificationExitWorld:
            if (isCurrent())
            {
                clearCurrent();
                mCurrent = true;
            }
            else
                mCurrent = false;

            if (mViewport)
            {
                mViewport->cameraRemoved(this);
                mViewport = nullptr;
            }
            break;
        default:
            break;
    }
}

void Camera3D::updateCamera()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->cameraSetTransform(mCamera, getCameraTransform());
}

void Camera3D::updateCameraMode()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;

    if (mProjection == CameraProjection::Perspective)
        server->cameraSetPerspective(mCamera, mFov, mNear, mFar);
    else
        server->cameraSetOrthogonal(mCamera, mSize, mNear, mFar);
}

void Camera3D::updateAttributes()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    if (mAttributesOverride) server->cameraSetAttributes(mCamera, mAttributes);
    else
        server->cameraClearAttributes(mCamera);
}

namespace
{

template<typename F>
void addAttributesField(const PropertyInfo& info, F CameraAttributesSettings::*member)
{
    NodeClassDB::addProperty<Camera3D>(
            info,
            [member](Camera3D* node, const PropertyValue& value)
            {
                CameraAttributesSettings attributes = node->getAttributes();
                attributes.*member = PropertyConvert<F>::from(value);
                node->setAttributes(attributes);
            },
            [member, type = info.type](const Camera3D* node)
            { return PropertyConvert<F>::to(node->getAttributes().*member, type); });
}

} // namespace

void Camera3D::bindProperties()
{
    using DB = NodeClassDB;
    using A = CameraAttributesSettings;
    DB::addProperty<Camera3D>(
            PropertyInfo::enumeration("projection", "Perspective,Orthogonal"),
            [](Camera3D* camera, const PropertyValue& value)
            {
                if (static_cast<CameraProjection>(value.asInt()) == CameraProjection::Orthogonal)
                    camera->setOrthogonal(camera->getSize(), camera->getNear(), camera->getFar());
                else
                    camera->setPerspective(camera->getFov(), camera->getNear(), camera->getFar());
            },
            &Camera3D::getProjection);
    DB::addProperty<Camera3D>(PropertyInfo(PropertyType::Bool, "current"), &Camera3D::setCurrent,
            &Camera3D::isCurrent);
    DB::addProperty<Camera3D>(PropertyInfo::range(PropertyType::Float, "fov", 1.0, 179.0, 0.1),
            &Camera3D::setFov, &Camera3D::getFov);
    DB::addProperty<Camera3D>(
            PropertyInfo::range(PropertyType::Float, "size", 0.001, 100.0, 0.001, true),
            &Camera3D::setSize, &Camera3D::getSize);
    DB::addProperty<Camera3D>(
            PropertyInfo::range(PropertyType::Float, "near", 0.001, 10.0, 0.001, true),
            &Camera3D::setNear, &Camera3D::getNear);
    DB::addProperty<Camera3D>(
            PropertyInfo::range(PropertyType::Float, "far", 0.01, 4000.0, 0.01, true),
            &Camera3D::setFar, &Camera3D::getFar);
    DB::addPropertyGroup<Camera3D>("Cull Mask", "cull_mask", true);
    DB::addProperty<Camera3D>(PropertyInfo::layers3DRender("cull_mask"), &Camera3D::setCullMask,
            &Camera3D::getCullMask);

    DB::addPropertyGroup<Camera3D>("Attributes", "attributes_");
    DB::addProperty<Camera3D>(
            PropertyInfo(PropertyType::Bool, "attributes_override", PropertyHint::GroupEnable),
            &Camera3D::setAttributesOverride, &Camera3D::isAttributesOverride);
    addAttributesField(
            PropertyInfo::range(PropertyType::Float, "attributes_dof_blur_amount", 0.0, 1.0, 0.01),
            &A::dofBlurAmount);

    DB::addPropertyGroup<Camera3D>("Attributes DOF Far", "attributes_dof_blur_far_");
    addAttributesField(PropertyInfo(PropertyType::Bool, "attributes_dof_blur_far_enabled",
                               PropertyHint::GroupEnable),
            &A::dofBlurFarEnabled);
    addAttributesField(PropertyInfo::range(PropertyType::Float, "attributes_dof_blur_far_distance",
                               0.01, 8192.0, 0.01),
            &A::dofBlurFarDistance);
    addAttributesField(PropertyInfo::range(PropertyType::Float,
                               "attributes_dof_blur_far_transition", -1.0, 8192.0, 0.01),
            &A::dofBlurFarTransition);

    DB::addPropertyGroup<Camera3D>("Attributes DOF Near", "attributes_dof_blur_near_");
    addAttributesField(PropertyInfo(PropertyType::Bool, "attributes_dof_blur_near_enabled",
                               PropertyHint::GroupEnable),
            &A::dofBlurNearEnabled);
    addAttributesField(PropertyInfo::range(PropertyType::Float, "attributes_dof_blur_near_distance",
                               0.01, 8192.0, 0.01),
            &A::dofBlurNearDistance);
    addAttributesField(PropertyInfo::range(PropertyType::Float,
                               "attributes_dof_blur_near_transition", -1.0, 8192.0, 0.01),
            &A::dofBlurNearTransition);
}

} // namespace prisma3d
