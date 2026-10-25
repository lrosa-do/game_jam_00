#pragma once

#include "prisma3d/editor/LightmapBakePass.h"
#include "prisma3d/editor/LightmapUnwrapJob.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorDocument;
class EditorSelection;
class HierarchyActions;
class Node;

class LightmapBaker
{
public:
    static constexpr const char* kLightmapNodeType = "LightmapGI";
    static constexpr const char* kDataExtension = ".lmdata";
    static constexpr const char* kFolderSuffix = "_lightmap";
    static constexpr const char* kUnwrapAction = "Unwrap UV2";
    static constexpr const char* kBakeAction = "Bake Lightmaps";
    static constexpr const char* kStaticAction = "Set Bake Mode";

    enum class State : std::uint8_t
    {
        Idle,
        Unwrapping,
        Baking
    };

    struct Settings
    {
        std::uint32_t resolution = 1024;
        std::uint32_t padding = 8;
        LightmapBakeSettings bake;
    };

    struct Summary
    {
        bool ok = false;
        std::uint32_t meshes = 0;
        std::uint32_t resolution = 0;
        std::uint32_t samples = 0;
        double seconds = 0.0;
        ct::String text;
    };

    LightmapBaker(EditorDocument& document, EditorSelection& selection, HierarchyActions& actions);
    ~LightmapBaker();
    LightmapBaker(const LightmapBaker&) = delete;
    LightmapBaker& operator=(const LightmapBaker&) = delete;

    Settings& settings();
    const Settings& settings() const;
    void applyPreset(bool draft);

    LightmapGI* lightmapNode() const;
    bool ensureLightmapNode(ct::String& error);
    DirectionalLight3D* sun() const;
    bool sunIsStatic() const;
    bool setSunStatic(ct::String& error);
    ct::Vector<MeshInstance3D*> meshes() const;
    bool markStatic(const ct::Vector<Node*>& nodes, ct::String& error);
    Math::Vec3 ambient() const;
    ct::String dataPath() const;

    bool startUnwrap(const ct::Vector<MeshInstance3D*>& nodes, ct::String& error);
    bool startBake(ct::String& error);
    void cancel();
    void update();

    State state() const;
    bool busy() const;
    float progress() const;
    const ct::String& message() const;
    const Summary& summary() const;
    const LightmapUnwrapResult& lastAtlas() const;

private:
    struct BakeItem
    {
        ct::String path;
        MeshId mesh;
        Math::Box bounds;
        Math::Mat4 transform;
        bool casts = true;
    };

    bool prepare(ct::String& error) const;
    void tickUnwrap();
    void tickBake();
    bool beginUnwrap(const ct::String& path, ct::String& error);
    bool applyUnwrap(const ct::String& path, const ct::Vector<SurfaceData>& surfaces,
            ct::String& error);
    bool beginBakeStage(ct::String& error);
    bool finishBake(ct::String& error);
    void finish(bool ok, const ct::String& text);
    void reset();
    Node* resolve(const ct::String& path) const;
    ct::String relativePath(const ct::String& absolute) const;
    ct::String sceneFolder() const;
    ct::String sceneStem() const;
    ct::String uniqueMeshPath(const ct::String& name) const;
    void collectMeshes(Node* node, ct::Vector<MeshInstance3D*>& out) const;
    DirectionalLight3D* findSun(Node* node, bool onlyStatic) const;
    LightmapGI* findLightmap(Node* node) const;
    LightmapBakeSettings effectiveSettings() const;

    EditorDocument& mDocument;
    EditorSelection& mSelection;
    HierarchyActions& mActions;
    Settings mSettings;
    State mState;
    bool mCancelRequested;
    bool mThenBake;
    ct::Vector<ct::String> mUnwrapQueue;
    ct::String mUnwrapping;
    LightmapUnwrapJob mJob;
    LightmapUnwrapResult mAtlas;
    ct::Vector<BakeItem> mBakeItems;
    ct::Vector<LightmapBakePass::Caster> mCasters;
    std::size_t mBakeIndex;
    LightmapBakePass mPass;
    ct::Vector<std::uint16_t> mLayers;
    std::uint32_t mLayerResolution;
    Math::Vec3 mSunDirection;
    Math::Vec3 mSunColor;
    LightmapBakeSettings mBakeSettings;
    std::uint32_t mUnwrapTotal;
    std::uint32_t mUnwrapDone;
    std::uint64_t mStartNanos;
    ct::String mMessage;
    Summary mSummary;
};

} // namespace prisma3d
