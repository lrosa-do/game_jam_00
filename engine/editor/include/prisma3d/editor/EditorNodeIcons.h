#pragma once

#include "prisma3d/editor/EditorView.h"
#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class EditorSelection;
class Node;
class Node3D;

class EditorNodeIcons
{
public:
    enum class Kind : int
    {
        None,
        Icon,
        Marker
    };

    struct Settings
    {
        float iconScale = 0.05f;
        float unselectedShade = 0.6f;
        float markerExtents = 0.25f;
        float markerDarken = 0.5f;
        float pickRadius = 8.0f;
        Math::Vec3 axisColor[3] = { Math::Vec3(0.96f, 0.20f, 0.32f),
            Math::Vec3(0.53f, 0.84f, 0.01f), Math::Vec3(0.16f, 0.55f, 0.96f) };
    };

    EditorNodeIcons();

    Settings& settings();
    const Settings& settings() const;

    bool load(const char* directory);
    bool loadDefault();
    bool loaded() const;
    static ct::String defaultDirectory();

    static Kind kindOf(const Node3D& node);
    static const char* iconName(const Node3D& node);
    static Math::Vec3 tint(const Node3D& node);

    static float billboardScale(const EditorView& view, const Math::Vec3& position);
    static bool iconHit(const EditorView& view, const Math::Vec3& position, float iconScale,
            float x, float y);
    static void markerSegments(const Math::Mat4& global, float extents,
            ct::Vector<Math::Vec3>& points);
    static bool markerHit(const EditorView& view, const Math::Mat4& global, float extents,
            float radius, float x, float y);

    static float pickDistance(const EditorView& view, const Node3D& node, const Settings& settings,
            float x, float y);

    void draw(BatchRenderer& batch, Node* root, const EditorSelection* selection,
            const EditorView& view);

    std::uint32_t iconCount() const;
    std::uint32_t markerCount() const;
    bool hasIcon(const char* name) const;

private:
    void collect(Node* node, ct::Vector<Node3D*>& out) const;

    Settings mSettings;
    TextureId mTexture;
    ct::HashMap<ct::String, Math::Vec4> mUv;
    bool mTried;
    std::uint32_t mIcons;
    std::uint32_t mMarkers;
};

} // namespace prisma3d
