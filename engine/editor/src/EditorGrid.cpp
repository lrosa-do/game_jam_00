#include "prisma3d/editor/EditorGrid.h"

#include "prisma3d/render/Batch.h"

#include <math.h>

namespace prisma3d
{

namespace
{

using Math::Vec3;
using Math::Vec4;

const float kPi = 3.14159265358979323846f;
const float kDegreesToRadians = kPi / 180.0f;
const float kEpsilon = 0.00001f;
const int kMaxBreaks = 40;

bool zeroApprox(float value)
{
    return fabsf(value) < kEpsilon;
}

struct Breaks
{
    float values[kMaxBreaks];
    int count = 0;

    void add(float value, float low, float high)
    {
        if (count >= kMaxBreaks || !(value > low && value < high)) return;
        int i = count++;
        while (i > 0 && values[i - 1] > value)
        {
            values[i] = values[i - 1];
            --i;
        }
        values[i] = value;
    }
};

} // namespace

EditorGrid::EditorGrid()
        : mGrid(true),
          mOrigin(true),
          mLines(0),
          mVertices(0)
{
}

EditorGrid::Settings& EditorGrid::settings()
{
    return mSettings;
}

const EditorGrid::Settings& EditorGrid::settings() const
{
    return mSettings;
}

void EditorGrid::setGridEnabled(bool enabled)
{
    mGrid = enabled;
}

bool EditorGrid::gridEnabled() const
{
    return mGrid;
}

void EditorGrid::setOriginEnabled(bool enabled)
{
    mOrigin = enabled;
}

bool EditorGrid::originEnabled() const
{
    return mOrigin;
}

std::uint32_t EditorGrid::lineCount() const
{
    return mLines;
}

std::uint32_t EditorGrid::vertexCount() const
{
    return mVertices;
}

float EditorGrid::smoothstep(float from, float to, float x)
{
    float t = (x - from) / (to - from);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

void EditorGrid::draw(BatchRenderer& batch, const EditorView& view)
{
    mLines = 0;
    mVertices = 0;
    if (!mGrid && !mOrigin) return;

    const BatchRenderer::Space space = batch.space();
    batch.setSpace(BatchRenderer::Space::World);
    batch.pushMatrix();
    batch.loadIdentity();
    batch.setTexture(batch.whiteTexture());
    batch.setDepthTest(true);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setCullFace(false);

    if (mGrid) drawGrid(batch, view);
    if (mOrigin) drawOrigin(batch, view);

    batch.popMatrix();
    batch.setDefault3DState();
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setColor(1.0f, 1.0f, 1.0f, 1.0f);
    batch.setSpace(space);
}

void EditorGrid::drawGrid(BatchRenderer& batch, const EditorView& view)
{
    const bool orthogonal = view.orthogonal();
    const Vec3 realCamera = view.position();
    Vec3 cameraPosition = realCamera;

    const int gridSize = mSettings.gridSize;
    const int steps = mSettings.primaryGridSteps;
    const bool enable[3] = { orthogonal || mSettings.planeXY, orthogonal || mSettings.planeYZ,
        orthogonal || mSettings.planeXZ };

    const float bias = mSettings.divisionLevelBias;
    int levelMax = mSettings.divisionLevelMax;
    int levelMin = mSettings.divisionLevelMin;
    if (levelMax < levelMin) return;
    if (steps != 10)
    {
        const float div = logf(static_cast<float>(steps)) / 2.302585092994045901094f;
        levelMax = static_cast<int>(static_cast<float>(levelMax) / div);
        levelMin = static_cast<int>(static_cast<float>(levelMin) / div);
    }

    const Vec3 forward = view.forward();
    const float origin = static_cast<float>(steps);

    for (int a = 0; a < 3; ++a)
    {
        if (!enable[a]) continue;
        const int b = (a + 1) % 3;
        const int c = (a + 2) % 3;

        Vec3 normal(0.0f, 0.0f, 0.0f);
        normal[c] = 1.0f;

        float cameraDistance = fabsf(cameraPosition[c]);
        if (orthogonal)
        {
            cameraDistance = view.projection.size / 2.0f;
            const float den = normal.Dot(forward);
            if (!zeroApprox(den))
            {
                const float dist = (normal.Dot(cameraPosition) - 0.0f) / den;
                if (dist <= kEpsilon) cameraPosition = cameraPosition - forward * dist;
            }
        }

        const float divisionLevel = logf(fabsf(cameraDistance)) / logf(origin) + bias;
        float clamped = divisionLevel;
        clamped = clamped < static_cast<float>(levelMin) ? static_cast<float>(levelMin) : clamped;
        clamped = clamped > static_cast<float>(levelMax) ? static_cast<float>(levelMax) : clamped;
        const float floored = floorf(clamped);
        const float decimals = clamped - floored;

        const float smallStep = powf(origin, floored);
        const float largeStep = smallStep * origin;
        const float centerA =
                largeStep * static_cast<float>(static_cast<int>(cameraPosition[a] / largeStep));
        const float centerB =
                largeStep * static_cast<float>(static_cast<int>(cameraPosition[b] / largeStep));

        const float beginA = centerA - static_cast<float>(gridSize) * smallStep;
        const float endA = centerA + static_cast<float>(gridSize) * smallStep;
        const float beginB = centerB - static_cast<float>(gridSize) * smallStep;
        const float endB = centerB + static_cast<float>(gridSize) * smallStep;

        float fadeSize = powf(origin, divisionLevel - 1.0f);
        const float minFade = powf(origin, static_cast<float>(levelMin));
        const float maxFade = powf(origin, static_cast<float>(levelMax));
        fadeSize = fadeSize < minFade ? minFade : (fadeSize > maxFade ? maxFade : fadeSize);
        const float gridFadeSize = static_cast<float>(gridSize - steps) * fadeSize;

        const float height = fabsf(realCamera[c]);
        const float angleFull = height / 0.2f;

        for (int i = -gridSize; i <= gridSize; ++i)
        {
            Vec4 color;
            if (i % steps == 0)
            {
                const Vec4& p = mSettings.primaryColor;
                const Vec4& s = mSettings.secondaryColor;
                color = p + (s - p) * decimals;
            }
            else
            {
                color = mSettings.secondaryColor;
                color.w = color.w * (1.0f - decimals);
            }

            for (int pass = 0; pass < 2; ++pass)
            {
                const int fixedAxis = pass == 0 ? a : b;
                const int lineAxis = pass == 0 ? b : a;
                const float position =
                        (pass == 0 ? centerA : centerB) + static_cast<float>(i) * smallStep;
                if (mOrigin && zeroApprox(position)) continue;
                const float low = pass == 0 ? beginB : beginA;
                const float high = pass == 0 ? endB : endA;

                const float foot = realCamera[lineAxis];
                const float offset = position - realCamera[fixedAxis];
                const float d0 = fabsf(offset);

                Breaks breaks;
                const float fadeFractions[5] = { 0.5f, 0.7f, 0.8f, 0.9f, 1.0f };
                for (int k = 0; k < 5; ++k)
                {
                    const float distance = gridFadeSize * fadeFractions[k];
                    if (distance <= d0) continue;
                    const float t = sqrtf(distance * distance - d0 * d0);
                    breaks.add(foot - t, low, high);
                    breaks.add(foot + t, low, high);
                }
                if (!orthogonal && height > kEpsilon)
                {
                    const float angleFactors[5] = { 1.0f, 1.5f, 2.2f, 3.2f, 4.0f };
                    for (int k = 0; k < 5; ++k)
                    {
                        const float s = angleFull * angleFactors[k];
                        const float planar = sqrtf(s * s - height * height);
                        if (planar <= d0) continue;
                        const float t = sqrtf(planar * planar - d0 * d0);
                        breaks.add(foot - t, low, high);
                        breaks.add(foot + t, low, high);
                    }
                }

                float points[kMaxBreaks + 2];
                int count = 0;
                points[count++] = low;
                for (int k = 0; k < breaks.count; ++k) points[count++] = breaks.values[k];
                points[count++] = high;

                float alphas[kMaxBreaks + 2];
                for (int k = 0; k < count; ++k)
                {
                    Vec3 world(0.0f, 0.0f, 0.0f);
                    world[fixedAxis] = position;
                    world[lineAxis] = points[k];
                    Vec3 onPlane = realCamera;
                    onPlane[c] = 0.0f;
                    const float planar = Vec3::Distance(world, onPlane);
                    float distFade = 1.0f - planar / gridFadeSize;
                    distFade = smoothstep(0.02f, 0.3f, distFade);
                    float angleDot = 0.0f;
                    if (orthogonal) angleDot = fabsf(forward.Dot(normal));
                    else
                        angleDot = fabsf((realCamera - world).Normalized().Dot(normal));
                    const float angleFade = smoothstep(0.05f, 0.2f, angleDot);
                    alphas[k] = color.w * distFade * angleFade;
                }

                for (int k = 0; k + 1 < count; ++k)
                {
                    if (alphas[k] < 1.0f / 255.0f && alphas[k + 1] < 1.0f / 255.0f) continue;
                    Vec3 from(0.0f, 0.0f, 0.0f);
                    Vec3 to(0.0f, 0.0f, 0.0f);
                    from[fixedAxis] = position;
                    to[fixedAxis] = position;
                    from[lineAxis] = points[k];
                    to[lineAxis] = points[k + 1];
                    batch.begin(BatchRenderer::ModeLines);
                    batch.setColor(color.x, color.y, color.z, alphas[k]);
                    batch.vertex3(from.x, from.y, from.z);
                    batch.setColor(color.x, color.y, color.z, alphas[k + 1]);
                    batch.vertex3(to.x, to.y, to.z);
                    batch.end();
                    ++mLines;
                    mVertices += 2;
                }
            }
        }
    }
}

void EditorGrid::drawThickLine(BatchRenderer& batch, const EditorView& view, const Vec3& from,
        const Vec3& to, const Vec4& color)
{
    const Vec3 eye = view.position();
    const Vec3 forward = view.forward();
    Vec3 a = from;
    Vec3 b = to;
    const float near = view.projection.zNear * 1.01f;
    float depthA = forward.Dot(a - eye);
    float depthB = forward.Dot(b - eye);
    if (!view.orthogonal())
    {
        if (depthA < near && depthB < near) return;
        if (depthA < near)
        {
            const float t = (near - depthA) / (depthB - depthA);
            a = a + (b - a) * t;
            depthA = near;
        }
        else if (depthB < near)
        {
            const float t = (near - depthB) / (depthA - depthB);
            b = b + (a - b) * t;
            depthB = near;
        }
    }

    const Vec3 direction = (b - a).Normalized();
    const float half = mSettings.originWidth * 0.5f;
    const float offsets[4] = { -half, -half + 1.0f, half - 1.0f, half };
    const float alphas[4] = { 0.0f, 1.0f, 1.0f, 0.0f };

    const float tanHalf = tanf(view.projection.fov * kDegreesToRadians * 0.5f);
    const float perPixelA = view.orthogonal() ? view.projection.size / view.height
                                              : 2.0f * depthA * tanHalf / view.height;
    const float perPixelB = view.orthogonal() ? view.projection.size / view.height
                                              : 2.0f * depthB * tanHalf / view.height;
    const Vec3 sideA = view.orthogonal() ? direction.Cross(forward).Normalized()
                                         : direction.Cross(a - eye).Normalized();
    const Vec3 sideB = view.orthogonal() ? direction.Cross(forward).Normalized()
                                         : direction.Cross(b - eye).Normalized();

    Vec3 pa[4];
    Vec3 pb[4];
    for (int k = 0; k < 4; ++k)
    {
        pa[k] = a + sideA * (offsets[k] * perPixelA);
        pb[k] = b + sideB * (offsets[k] * perPixelB);
    }
    batch.begin(BatchRenderer::ModeTriangles);
    for (int k = 0; k < 3; ++k)
    {
        batch.setColor(color.x, color.y, color.z, color.w * alphas[k]);
        batch.vertex3(pa[k].x, pa[k].y, pa[k].z);
        batch.setColor(color.x, color.y, color.z, color.w * alphas[k + 1]);
        batch.vertex3(pa[k + 1].x, pa[k + 1].y, pa[k + 1].z);
        batch.setColor(color.x, color.y, color.z, color.w * alphas[k]);
        batch.vertex3(pb[k].x, pb[k].y, pb[k].z);

        batch.setColor(color.x, color.y, color.z, color.w * alphas[k + 1]);
        batch.vertex3(pa[k + 1].x, pa[k + 1].y, pa[k + 1].z);
        batch.setColor(color.x, color.y, color.z, color.w * alphas[k + 1]);
        batch.vertex3(pb[k + 1].x, pb[k + 1].y, pb[k + 1].z);
        batch.setColor(color.x, color.y, color.z, color.w * alphas[k]);
        batch.vertex3(pb[k].x, pb[k].y, pb[k].z);
    }
    batch.end();
    mVertices += 18;
}

void EditorGrid::drawOrigin(BatchRenderer& batch, const EditorView& view)
{
    const float extent = view.projection.zFar * 2.0f + view.position().Length();
    for (int i = 0; i < 3; ++i)
    {
        Vec3 axis(0.0f, 0.0f, 0.0f);
        axis[i] = extent;
        drawThickLine(batch, view, -axis, axis, mSettings.axisColor[i]);
    }
}

} // namespace prisma3d
