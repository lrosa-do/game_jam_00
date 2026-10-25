#pragma once

#include "prisma3d/editor/EditorView.h"

#include "mathc.h"

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;

class EditorGrid
{
public:
    struct Settings
    {
        Math::Vec4 primaryColor = Math::Vec4(0.56f, 0.56f, 0.56f, 0.5f);
        Math::Vec4 secondaryColor = Math::Vec4(0.38f, 0.38f, 0.38f, 0.5f);
        int gridSize = 200;
        int primaryGridSteps = 8;
        int divisionLevelMax = 2;
        int divisionLevelMin = 0;
        float divisionLevelBias = -0.2f;
        bool planeXY = false;
        bool planeYZ = false;
        bool planeXZ = true;
        float originWidth = 3.0f;
        Math::Vec4 axisColor[3] = { Math::Vec4(0.96f, 0.20f, 0.32f, 1.0f),
            Math::Vec4(0.53f, 0.84f, 0.01f, 1.0f), Math::Vec4(0.16f, 0.55f, 0.96f, 1.0f) };
    };

    EditorGrid();

    Settings& settings();
    const Settings& settings() const;
    void setGridEnabled(bool enabled);
    bool gridEnabled() const;
    void setOriginEnabled(bool enabled);
    bool originEnabled() const;

    void draw(BatchRenderer& batch, const EditorView& view);

    std::uint32_t lineCount() const;
    std::uint32_t vertexCount() const;

    static float smoothstep(float from, float to, float x);

private:
    void drawGrid(BatchRenderer& batch, const EditorView& view);
    void drawOrigin(BatchRenderer& batch, const EditorView& view);
    void drawThickLine(BatchRenderer& batch, const EditorView& view, const Math::Vec3& a,
            const Math::Vec3& b, const Math::Vec4& color);

    Settings mSettings;
    bool mGrid;
    bool mOrigin;
    std::uint32_t mLines;
    std::uint32_t mVertices;
};

} // namespace prisma3d
