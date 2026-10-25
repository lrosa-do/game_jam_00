#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"

namespace prisma3d
{

class Camera3D;

class Viewport : public Node
{
public:
    PRISMA3D_NODE_CLASS(Viewport, Node);
    static void bindProperties();

    Viewport();
    ~Viewport() override;

    ViewportId getViewportId() const;
    ScenarioId getScenario() const;

    void setSize(std::uint32_t width, std::uint32_t height);
    std::uint32_t getWidth() const;
    std::uint32_t getHeight() const;
    float getAspect() const;
    Math::Vec2 getMousePosition() const;
    void setClearColor(const Math::Vec4& color);
    Math::Vec4 getClearColor() const;
    void setMsaa3D(ViewportMsaa msaa);
    ViewportMsaa getMsaa3D() const;
    void setScaling3D(float scale);
    float getScaling3D() const;
    // A fixed size for the 3D picture (0 in either goes back to the scale), fitted to the
    // viewport by the aspect mode (Scaling3DAspect).
    void setScaling3DSize(std::uint32_t width, std::uint32_t height);
    std::uint32_t getScaling3DWidth() const;
    std::uint32_t getScaling3DHeight() const;
    void setScaling3DAspect(int aspect);
    int getScaling3DAspect() const;
    // 0 Bilinear, 1 FSR 1.0; the FSR sharpness is in stops, 0 the sharpest, 2 the softest.
    void setScaling3DMode(int mode);
    int getScaling3DMode() const;
    void setFsrSharpness(float sharpness);
    float getFsrSharpness() const;
    // The rectangle the 3D picture fills, in viewport pixels (x, y, width, height).
    Math::Vec4 get3DRect() const;
    // 0 off, 1 FXAA, 2 SMAA.
    void setScreenSpaceAA(int mode);
    int getScreenSpaceAA() const;
    void setMeshLodThreshold(float pixels);
    float getMeshLodThreshold() const;
    std::uint32_t getMsaaSamples() const;
    void setPositionalShadowAtlasSize(std::uint32_t size, bool sixteenBits = false);
    std::uint32_t getPositionalShadowAtlasSize() const;
    bool getPositionalShadowAtlasSixteenBits() const;
    void setPositionalShadowAtlasQuadrantSubdivision(std::uint32_t quadrant,
            std::uint32_t subdivision);
    std::uint32_t getPositionalShadowAtlasQuadrantSubdivision(std::uint32_t quadrant) const;

    Camera3D* getCamera3D() const;

    static Viewport* findViewport(const Node* node);

protected:
    void notificationHandler(int what) override;

private:
    friend class Camera3D;

    void cameraSetCurrent(Camera3D* camera);
    void cameraClearCurrent(Camera3D* camera);
    void cameraRemoved(Camera3D* camera);

    ViewportId mViewport;
    ScenarioId mScenario;
    std::uint32_t mWidth;
    std::uint32_t mHeight;
    Math::Vec4 mClearColor;
    ViewportMsaa mMsaa3D;
    float mScaling3D = 1.0f;
    std::uint32_t mScaling3DWidth = 0;
    std::uint32_t mScaling3DHeight = 0;
    int mScaling3DAspect = 0;
    int mScaling3DMode = 0;
    float mFsrSharpness = 0.2f;
    int mScreenSpaceAA = 0;
    float mMeshLodThreshold = 1.0f;
    std::uint32_t mPositionalShadowAtlasSize;
    bool mPositionalShadowAtlasSixteenBits;
    std::uint32_t mPositionalShadowAtlasSubdivision[4];
    Camera3D* mCamera;
    ct::Vector<Camera3D*> mCameras;
};

class Window : public Viewport
{
public:
    PRISMA3D_NODE_CLASS(Window, Viewport);
    static void bindProperties();

    Window();
    ~Window() override;

protected:
    void notificationHandler(int what) override;

private:
    void syncSize();
};

} // namespace prisma3d
