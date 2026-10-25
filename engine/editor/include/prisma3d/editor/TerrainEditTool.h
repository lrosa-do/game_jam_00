#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/terrain/Terrain.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/ComboBox.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>

#include <ct/ptr.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class EditorSelection;
class Forest3D;
class Grass3D;
class Node;
class Terrain3D;

class TerrainEditTool
{
public:
    enum class Mode : int
    {
        Raise,
        Smooth,
        Flatten,
        Paint,
        Colour,
        Grass,
        Plant,
        kCount
    };

    enum class Kind : int
    {
        None,
        Terrain,
        Grass,
        Forest
    };

    struct InstanceRecord
    {
        int species = 0;
        Math::Mat4 transform = Math::Mat4::Identity();
        bool onTerrain = false;
    };

    static constexpr float kDefaultRadius = 25.0f;
    static constexpr float kDefaultStrength = 0.5f;
    static constexpr float kDefaultSpacing = 4.0f;
    static constexpr float kMinRadius = 0.5f;
    static constexpr float kMaxRadius = 10000.0f;
    static constexpr float kMinStrength = 0.01f;
    static constexpr float kMaxStrength = 10.0f;
    static constexpr float kMinSpacing = 0.0f;
    static constexpr float kMaxSpacing = 1000.0f;
    static constexpr float kHeightStrength = 250.0f;
    static constexpr float kBlendStrength = 1.0f;
    static constexpr float kColourStrength = 2.0f;
    static constexpr float kGrassStrength = 4.0f;
    static constexpr float kPlantRate = 20.0f;
    static constexpr int kPlantAttempts = 8;
    static constexpr float kUpdateInterval = 1.0f / 20.0f;
    static constexpr const char* kNoTerrainText = "No Terrain3D in the scene";
    static constexpr const char* kActionName = "Terrain Brush";
    static constexpr const char* kColourActionName = "Colour Brush";
    static constexpr const char* kGrassActionName = "Grass Brush";
    static constexpr const char* kPlantActionName = "Plant Brush";
    static constexpr const char* kFillActionName = "Fill Forest";
    static constexpr const char* kClearActionName = "Clear Forest";

    TerrainEditTool();
    ~TerrainEditTool();
    TerrainEditTool(const TerrainEditTool&) = delete;
    TerrainEditTool& operator=(const TerrainEditTool&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    ig::retained::Widget* bar() const;

    void setChangeListener(EditorChangeListener* listener);
    void setSceneRoot(Node* root);
    void setSelection(const EditorSelection* selection);

    void setActive(bool active);
    bool active() const;
    void setMode(Mode mode);
    Mode mode() const;
    static const char* modeName(Mode mode);
    static Kind kindOf(Mode mode);
    void setRadius(float radius);
    float radius() const;
    void setStrength(float strength);
    float strength() const;
    void setSpacing(float spacing);
    float spacing() const;
    void setLayer(int layer);
    int layer() const;
    void setSpecies(int species);
    int species() const;
    void setColour(const Math::Vec4& colour);
    const Math::Vec4& colour() const;

    Kind kind() const;
    Terrain3D* target() const;
    Grass3D* grass() const;
    Forest3D* forest() const;
    bool stroking() const;
    bool hovering() const;
    const Math::Vec3& hitPosition() const;
    unsigned int strokeCount() const;

    bool handleEvent(const InputEvent& event, float x, float y, bool navigating);
    void fillForest();
    void clearForest();
    ig::retained::Button* fillButton() const;
    ig::retained::Button* clearButton() const;
    void update(float delta, const EditorView& view);
    void draw(BatchRenderer& batch);

    static void applyInstances(Forest3D* forest, const ct::Vector<InstanceRecord>& records);
    static void snapshotInstances(const Forest3D* forest, ct::Vector<InstanceRecord>& out);

private:
    struct Link
    {
        TerrainEditTool* owner;
    };

    struct Block
    {
        TerrainRect rect;
        ct::Vector<float> before;
        ct::Vector<float> after;
    };

    void refreshTarget();
    void syncBar();
    void syncLayers();
    void syncSpecies();
    bool computeHit(const EditorView& view);
    void beginStroke();
    void applyBrush(float delta);
    void applyGrass(float delta);
    void applyColour(float delta);
    bool diffColour(TerrainRect& rect, ct::Vector<std::uint8_t>& before,
            ct::Vector<std::uint8_t>& after) const;
    static void applyColourBlock(Terrain3D* terrain, const TerrainRect& rect,
            const ct::Vector<std::uint8_t>& values);
    void applyPlant(float delta);
    void endStroke();
    void cancelStroke();
    const float* currentData(std::size_t& size) const;
    void takeSnapshot();
    bool diffSnapshot(Block& block) const;
    static void applyBlock(Terrain3D* terrain, Mode mode, int layer, const TerrainRect& rect,
            const ct::Vector<float>& values);
    static void applyMask(Grass3D* grass, const ct::Vector<std::uint8_t>& pixels);
    void ensureGrassExtent();
    float nextRandom();
    void modeToggled(int id, bool checked);
    void recordInstances(const char* action, ct::Vector<InstanceRecord>& before);

    ct::Rc<Link> mLink;
    EditorChangeListener* mListener;
    Node* mRoot;
    const EditorSelection* mSelection;
    Terrain3D* mTarget;
    Grass3D* mGrass;
    Forest3D* mForest;
    ig::retained::BoxLayout* mBar;
    ig::retained::ToolBar* mModes;
    int mModeButtons[static_cast<int>(Mode::kCount)];
    ig::retained::Label* mLayerLabel;
    ig::retained::ComboBox* mLayerBox;
    ig::retained::Label* mSpeciesLabel;
    ig::retained::ComboBox* mSpeciesBox;
    NumberField* mRadiusField;
    NumberField* mStrengthField;
    ig::retained::Label* mSpacingLabel;
    NumberField* mSpacingField;
    ColorSwatch* mSwatch;
    ig::retained::Button* mFill;
    ig::retained::Button* mClear;
    ig::retained::Label* mStatus;
    Mode mMode;
    float mRadius;
    float mStrength;
    float mSpacing;
    Math::Vec4 mColour;
    int mLayer;
    int mLayerCount;
    int mSpecies;
    int mSpeciesCount;
    bool mActive;
    bool mInvert;
    bool mStroking;
    bool mStrokeMoved;
    bool mHovering;
    bool mSyncing;
    float mMouseX;
    float mMouseY;
    float mSinceUpdate;
    float mFlattenHeight;
    float mPlantAccumulator;
    std::uint32_t mRandom;
    Math::Vec3 mHit;
    ct::Vector<float> mSnapshot;
    ct::Vector<std::uint8_t> mMaskSnapshot;
    ct::Vector<std::uint8_t> mColourSnapshot;
    ct::Vector<InstanceRecord> mInstanceSnapshot;
    unsigned int mStrokes;
};

} // namespace prisma3d
