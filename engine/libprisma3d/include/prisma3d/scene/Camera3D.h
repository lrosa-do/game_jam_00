#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class Viewport;

class Camera3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Camera3D, Node3D);
    static void bindProperties();

    Camera3D();
    ~Camera3D() override;

    CameraId getCameraId() const;

    void setPerspective(float fovyDegrees, float zNear, float zFar);
    void setOrthogonal(float size, float zNear, float zFar);
    CameraProjection getProjection() const;
    void setFov(float fovyDegrees);
    float getFov() const;
    void setSize(float size);
    float getSize() const;
    void setNear(float zNear);
    float getNear() const;
    void setFar(float zFar);
    float getFar() const;
    void setCullMask(std::uint32_t mask);
    std::uint32_t getCullMask() const;
    void setAttributes(const CameraAttributesSettings& attributes);
    const CameraAttributesSettings& getAttributes() const;
    void setAttributesOverride(bool enabled);
    bool isAttributesOverride() const;

    void makeCurrent();
    void clearCurrent(bool enableNext = true);
    void setCurrent(bool current);
    bool isCurrent() const;

    Math::Mat4 getCameraTransform() const;
    Math::Mat4 getCameraProjection() const;

    Math::Vec3 projectRayNormal(const Math::Vec2& point) const;
    Math::Vec3 projectLocalRayNormal(const Math::Vec2& point) const;
    Math::Vec3 projectRayOrigin(const Math::Vec2& point) const;
    Math::Ray projectRay(const Math::Vec2& point) const;
    Math::Ray projectMouseRay() const;
    Math::Vec3 projectPosition(const Math::Vec2& point, float zDepth) const;
    Math::Vec2 unprojectPosition(const Math::Vec3& position) const;
    bool pickDepth(const Math::Vec2& point, Math::Vec3& position) const;
    bool isPositionBehind(const Math::Vec3& position) const;
    bool isPositionInFrustum(const Math::Vec3& position) const;
    Viewport* getViewport() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    friend class Viewport;

    void updateCamera();
    void updateCameraMode();
    void updateAttributes();

    CameraId mCamera;
    Viewport* mViewport;
    CameraProjection mProjection;
    float mFov;
    float mSize;
    float mNear;
    float mFar;
    std::uint32_t mCullMask;
    CameraAttributesSettings mAttributes;
    bool mAttributesOverride;
    bool mCurrent;
};

} // namespace prisma3d
