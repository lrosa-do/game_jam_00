#pragma once

#include "mathc.h"

#include <prisma/rhi/Driver.h>

#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

struct FloatRect
{
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

Math::Vec4 fontGlyphUVRect(unsigned char code);

class BatchRenderer
{
public:
    struct Config
    {
        std::size_t maxVertices = 32768;
        std::size_t stackDepth = 32;
        std::size_t uniformSlots = 16;
    };

    struct Stats
    {
        std::size_t drawCalls = 0;
        std::size_t verticesDrawn = 0;
        std::size_t indicesDrawn = 0;
        std::size_t textureSwitches = 0;
    };

    enum class BlendMode
    {
        Alpha,
        Additive,
        Multiplied,
        AddColors,
        SubtractColors
    };

    enum class Space
    {
        Screen,
        World
    };

    BatchRenderer();
    ~BatchRenderer();
    BatchRenderer(const BatchRenderer&) = delete;
    BatchRenderer& operator=(const BatchRenderer&) = delete;

    bool init(prisma::Driver* driver);
    bool init(prisma::Driver* driver, const Config& config);
    void shutdown();

    void setSpace(Space space);
    Space space() const;

    void pushMatrix();
    void popMatrix();
    void loadIdentity();
    void translate(float x, float y, float z = 0.0f);
    void rotate(float angleDeg, float axisX, float axisY, float axisZ);
    void scale(float x, float y, float z = 1.0f);

    void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
    void setColor(float r, float g, float b, float a = 1.0f);
    void setTexture(prisma::TextureHandle texture);
    void setBlendMode(BlendMode mode);
    void setTexcoord(float u, float v);
    void setDepthTest(bool enabled)
    {
        mDepthTestEnabled = enabled;
    }
    void setDepthWrite(bool enabled)
    {
        mDepthWriteEnabled = enabled;
    }
    void setBlend(bool enabled)
    {
        mBlendEnabled = enabled;
    }
    void setCullFace(bool enabled)
    {
        mCullFaceEnabled = enabled;
    }
    void setDefault3DState();

    void setClipRect(float x, float y, float width, float height);
    void setClipRect(const FloatRect& rect);
    void clearClipRect();
    bool isClipEnabled() const
    {
        return mClipEnabled;
    }
    const FloatRect& getClipRect() const
    {
        return mClipRect;
    }

    void begin(int mode);
    void end();
    void vertex2(float x, float y);
    void vertex3(float x, float y, float z);

    void drawLine(float x0, float y0, float x1, float y1);
    void drawThickLine(float x0, float y0, float x1, float y1, float thickness);
    void drawTriangle(float x1, float y1, float x2, float y2, float x3, float y3);
    void drawRect(float x, float y, float width, float height, bool fill = true);
    void drawQuad(float x, float y, float width, float height);
    void drawCircle(float cx, float cy, float radius, int segments = 32);
    void drawEllipse(float cx, float cy, float rx, float ry, bool fill = true, int segments = 32);
    void drawArc(float cx, float cy, float radius, float startDeg, float endDeg, int segments = 32);
    void drawRing(float cx, float cy, float rInner, float rOuter, bool fill = true,
            int segments = 32);
    void drawPolygon(float cx, float cy, int sides, float radius, float rotDeg = 0.0f,
            bool fill = true);
    void drawPolyline(const float* xyPairs, int pointCount);

    void drawLine3D(float x0, float y0, float z0, float x1, float y1, float z1);
    void drawTriangle3D(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c);
    void drawTriangle3D(const Math::Vec3& a, const Math::Vec2& uvA, const Math::Vec3& b,
            const Math::Vec2& uvB, const Math::Vec3& c, const Math::Vec2& uvC);
    void drawTriangle3D(const Math::Vec3& a, const Math::Vec2& uvA, std::uint32_t colorA,
            const Math::Vec3& b, const Math::Vec2& uvB, std::uint32_t colorB, const Math::Vec3& c,
            const Math::Vec2& uvC, std::uint32_t colorC);

    void drawWireBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void drawWireSphere(float cx, float cy, float cz, float radius, int segments = 24);
    void drawWireCylinder(float cx, float cy, float cz, float radius, float height,
            int segments = 24);
    void drawWireCapsule(float cx, float cy, float cz, float radius, float height,
            int segments = 24);

    void drawSolidBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void drawSolidSphere(float cx, float cy, float cz, float radius, int rings = 12,
            int segments = 24);
    void drawSolidCylinder(float cx, float cy, float cz, float radius, float height,
            int segments = 24);
    void drawSolidCapsule(float cx, float cy, float cz, float radius, float height, int rings = 8,
            int segments = 24);

    void drawAxis(float x, float y, float z, float size = 1.0f);
    void drawWireGrid(float y, int slices, float spacing, bool axes = true);

    void drawTexture(prisma::TextureHandle texture, float dstX, float dstY, float dstW, float dstH,
            float srcX = 0.0f, float srcY = 0.0f, float srcW = 0.0f, float srcH = 0.0f,
            float pivotX = 0.0f, float pivotY = 0.0f, float rotationDeg = 0.0f);

    void drawText(float x, float y, float size, const char* text);
    float textWidth(float size, const char* text) const;
    prisma::TextureHandle fontTexture() const
    {
        return mFontTexture;
    }
    prisma::TextureHandle whiteTexture() const
    {
        return mWhiteTexture;
    }

    bool empty(Space space) const;
    void clear();

    void upload();
    void setUniforms(std::uint32_t slot, const Math::Mat4& projection, bool linearColors);
    void draw(Space space, std::uint32_t slot, std::uint32_t samples = 1,
            prisma::TextureFormat screenTarget = prisma::TextureFormat::None);

    static Math::Mat4 screenProjection(float width, float height);

    void resetStats();
    const Stats& getStats() const
    {
        return mStats;
    }

    static unsigned int packColor(unsigned char r, unsigned char g, unsigned char b,
            unsigned char a);
    static void unpackColor(unsigned int packed, unsigned char& r, unsigned char& g,
            unsigned char& b, unsigned char& a);

    static constexpr int ModeLines = 0x0001;
    static constexpr int ModeTriangles = 0x0004;
    static constexpr int ModeQuads = 0x0007;

private:
#pragma pack(push, 1)
    struct Vertex
    {
        float x, y, z;
        float u, v;
        unsigned char r, g, b, a;
    };
#pragma pack(pop)

    struct DrawCall
    {
        int mode;
        std::size_t vertexCount;
        std::size_t vertexAlignment;
        std::uint32_t firstIndex;
        prisma::TextureHandle texture;
        BlendMode blendMode;
        bool depthTest;
        bool depthWrite;
        bool blend;
        bool cullFace;
        Space space;
    };

    prisma::PipelineHandle pipelineFor(const DrawCall& call, std::uint32_t samples,
            prisma::TextureFormat screenTarget);
    void submitVertex(float x, float y, float z);
    void submitVertex(float x, float y, float z, float u, float v);
    void applyTransform(float& x, float& y, float& z);
    bool ensureBuffer(prisma::BufferHandle& buffer, std::uint32_t& capacity, std::uint32_t size,
            prisma::BufferUsage usage, prisma::IndexFormat format, const char* name);

    struct ClipVertex
    {
        float x, y, u, v;
    };
    int clipPolygonToRect(const ClipVertex* in, int inCount, ClipVertex* out) const;
    bool clipSegmentToRect(float& x0, float& y0, float& x1, float& y1) const;
    void submitClippedQuad(float x0, float y0, float x1, float y1, float x2, float y2, float x3,
            float y3);
    void submitClippedLine(float x0, float y0, float x1, float y1);
    void emitLine(float x0, float y0, float x1, float y1);
    void emitTriangle(float x0, float y0, float x1, float y1, float x2, float y2);
    void emitTexturedTriangle(float x0, float y0, float u0, float v0, float x1, float y1, float u1,
            float v1, float x2, float y2, float u2, float v2);

    struct SpaceScope
    {
        SpaceScope(BatchRenderer& batch, Space space)
                : batch(batch),
                  previous(batch.mSpace)
        {
            batch.mSpace = space;
        }
        ~SpaceScope()
        {
            batch.mSpace = previous;
        }
        BatchRenderer& batch;
        Space previous;
    };

    Config mConfig;
    Stats mStats;

    ct::Vector<Vertex> mVertices;
    ct::Vector<std::uint32_t> mIndices;
    ct::Vector<DrawCall> mDrawCalls;

    ct::Vector<Math::Mat4> mMatrixStack;
    Math::Mat4 mCurrentMatrix;

    unsigned int mCurrentColor;
    prisma::TextureHandle mCurrentTexture;
    float mCurrentTexcoord[2];
    int mCurrentMode;
    bool mInBeginEnd;
    BlendMode mCurrentBlendMode;
    Space mSpace;

    bool mDepthTestEnabled;
    bool mDepthWriteEnabled;
    bool mBlendEnabled;
    bool mCullFaceEnabled;

    bool mClipEnabled;
    FloatRect mClipRect;

    struct CachedPipeline
    {
        std::uint32_t key;
        prisma::PipelineHandle pipeline;
    };

    prisma::Driver* mDriver;
    prisma::ShaderHandle mVertexShader;
    prisma::ShaderHandle mFragmentShader;
    prisma::BufferHandle mVertexBuffer;
    prisma::BufferHandle mIndexBuffer;
    prisma::BufferHandle mUniformBuffer;
    std::uint32_t mVertexCapacity;
    std::uint32_t mIndexCapacity;
    std::uint32_t mUniformStride;
    prisma::TextureHandle mWhiteTexture;
    prisma::TextureHandle mFontTexture;
    prisma::SamplerHandle mSampler;
    ct::Vector<CachedPipeline> mPipelines;
};

} // namespace prisma3d
