#pragma once

#include "prisma3d/render/RenderingServer.h"

#include <cstdint>

namespace prisma3d
{

class MaterialPreview
{
public:
    enum class Shape : std::uint8_t
    {
        Sphere,
        Cube
    };

    static constexpr std::uint32_t kSize = 256;

    explicit MaterialPreview(RenderingServer& server);
    ~MaterialPreview();
    MaterialPreview(const MaterialPreview&) = delete;
    MaterialPreview& operator=(const MaterialPreview&) = delete;

    void setMaterial(MaterialId material);
    MaterialId material() const;
    void setShape(Shape shape);
    Shape shape() const;

    void setVisible(bool visible);
    void invalidate();
    bool update();

    TextureId texture() const;
    unsigned int renderCount() const;
    bool readPixel(int x, int y, unsigned char* rgba) const;

private:
    RenderingServer& mServer;
    ScenarioId mScenario;
    ViewportId mViewport;
    CameraId mCamera;
    LightId mKey;
    LightId mFill;
    InstanceId mInstance;
    MeshId mMeshes[2];
    MaterialId mMaterial;
    std::uint32_t mVersion;
    unsigned int mRenders;
    Shape mShape;
    bool mVisible;
    bool mDirty;
};

} // namespace prisma3d
