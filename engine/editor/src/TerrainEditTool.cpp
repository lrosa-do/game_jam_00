#include "prisma3d/editor/TerrainEditTool.h"

#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/terrain/TerrainBrush.h"
#include "prisma3d/terrain/TerrainLayerBlendMap.h"

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

using Math::Vec3;

namespace
{

const float kPi = 3.14159265358979f;
const int kRingSegments = 64;
const float kRingLift = 0.15f;
const float kBarPadding = 4.0f;
const float kBarSpacing = 6.0f;
const float kFieldWidth = 72.0f;
const float kComboWidth = 110.0f;

const char* const kModeNames[static_cast<int>(TerrainEditTool::Mode::kCount)] = { "Raise", "Smooth",
    "Flatten", "Paint", "Colour", "Grass", "Plant" };
const char* const kModeTooltips[static_cast<int>(TerrainEditTool::Mode::kCount)] = {
    "Raise (Shift lowers)", "Smooth", "Flatten to the height under the first click",
    "Paint the chosen layer (Shift erases)", "Tint the global colour map with the chosen colour",
    "Paint grass (Shift erases)", "Plant the chosen species (Shift removes)"
};
const char* const kModeIcons[static_cast<int>(TerrainEditTool::Mode::kCount)] = { "MoveUp",
    "Curve3D", "Grid", "StandardMaterial3D", "Texture2D", "GPUParticles3D", "Add" };

const float kModeColors[static_cast<int>(TerrainEditTool::Mode::kCount)][3] = {
    { 0.45f, 0.95f, 0.45f }, { 0.45f, 0.75f, 1.0f }, { 1.0f, 0.9f, 0.35f }, { 1.0f, 0.6f, 0.25f },
    { 1.0f, 1.0f, 1.0f }, { 0.6f, 1.0f, 0.3f }, { 0.8f, 0.55f, 0.3f }
};

struct ColourEdit
{
    TerrainRect rect;
    ct::Vector<std::uint8_t> before;
    ct::Vector<std::uint8_t> after;
};

struct BlockEdit
{
    TerrainEditTool::Mode mode;
    int layer;
    TerrainRect rect;
    ct::Vector<float> before;
    ct::Vector<float> after;
};

struct MaskEdit
{
    ct::Vector<std::uint8_t> before;
    ct::Vector<std::uint8_t> after;
};

struct PlantEdit
{
    ct::Vector<TerrainEditTool::InstanceRecord> before;
    ct::Vector<TerrainEditTool::InstanceRecord> after;
};

bool sameBytes(const ct::Vector<std::uint8_t>& a, const ct::Vector<std::uint8_t>& b)
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] != b[i]) return false;
    return true;
}

} // namespace

TerrainEditTool::TerrainEditTool()
        : mLink(ct::make_rc<Link>(Link{ this })),
          mListener(nullptr),
          mRoot(nullptr),
          mSelection(nullptr),
          mTarget(nullptr),
          mGrass(nullptr),
          mForest(nullptr),
          mBar(nullptr),
          mModes(nullptr),
          mModeButtons{},
          mLayerLabel(nullptr),
          mLayerBox(nullptr),
          mSpeciesLabel(nullptr),
          mSpeciesBox(nullptr),
          mRadiusField(nullptr),
          mStrengthField(nullptr),
          mSpacingLabel(nullptr),
          mSpacingField(nullptr),
          mSwatch(nullptr),
          mFill(nullptr),
          mClear(nullptr),
          mStatus(nullptr),
          mMode(Mode::Raise),
          mRadius(kDefaultRadius),
          mStrength(kDefaultStrength),
          mSpacing(kDefaultSpacing),
          mColour(0.5f, 0.5f, 0.5f, 1.0f),
          mLayer(1),
          mLayerCount(-1),
          mSpecies(0),
          mSpeciesCount(-1),
          mActive(false),
          mInvert(false),
          mStroking(false),
          mStrokeMoved(false),
          mHovering(false),
          mSyncing(false),
          mMouseX(0.0f),
          mMouseY(0.0f),
          mSinceUpdate(0.0f),
          mFlattenHeight(0.0f),
          mPlantAccumulator(0.0f),
          mRandom(0x9E3779B9u),
          mHit(0.0f, 0.0f, 0.0f),
          mStrokes(0)
{
    for (int& id: mModeButtons) id = -1;
}

TerrainEditTool::~TerrainEditTool()
{
    mLink->owner = nullptr;
}

bool TerrainEditTool::attach(rt::Widget* parent)
{
    if (mBar || !parent) return false;
    mBar = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mBar->setPadding(kBarPadding);
    mBar->setSpacing(kBarSpacing);
    mBar->setVisible(false);

    mModes = mBar->createChild<rt::ToolBar>();
    mModes->setDraggable(false);
    mModes->setButtonSize(24.0f);
    mModes->setIconPadding(4.0f);
    for (int i = 0; i < static_cast<int>(Mode::kCount); ++i)
    {
        mModeButtons[i] = mModes->addToggle(EditorIcons::get(kModeIcons[i]), kModeTooltips[i],
                i == static_cast<int>(mMode));
        mModes->setItemDraggable(mModeButtons[i], false);
    }

    mLayerLabel = mBar->createChild<rt::Label>(ct::String("Layer"));
    mLayerBox = mBar->createChild<rt::ComboBox>();
    mLayerBox->setSize(kComboWidth, 0.0f);
    mLayerBox->setTooltip("Layer painted by the brush; the first layer is the base");

    mSpeciesLabel = mBar->createChild<rt::Label>(ct::String("Species"));
    mSpeciesBox = mBar->createChild<rt::ComboBox>();
    mSpeciesBox->setSize(kComboWidth, 0.0f);
    mSpeciesBox->setTooltip("Species planted by the brush");

    mBar->createChild<rt::Label>(ct::String("Radius"));
    mRadiusField = mBar->createChild<NumberField>("");
    mRadiusField->setSize(kFieldWidth, 0.0f);
    mRadiusField->setRange(kMinRadius, kMaxRadius);
    mRadiusField->setValue(mRadius);
    mRadiusField->setTooltip("Brush radius in metres");

    mBar->createChild<rt::Label>(ct::String("Strength"));
    mStrengthField = mBar->createChild<NumberField>("");
    mStrengthField->setSize(kFieldWidth, 0.0f);
    mStrengthField->setRange(kMinStrength, kMaxStrength);
    mStrengthField->setValue(mStrength);
    mStrengthField->setTooltip("Brush strength");

    mSpacingLabel = mBar->createChild<rt::Label>(ct::String("Spacing"));
    mSpacingField = mBar->createChild<NumberField>("");
    mSpacingField->setSize(kFieldWidth, 0.0f);
    mSpacingField->setRange(kMinSpacing, kMaxSpacing);
    mSpacingField->setValue(mSpacing);
    mSpacingField->setTooltip("Minimum distance between planted trees in metres");

    mSwatch = mBar->createChild<ColorSwatch>();
    mSwatch->setAlphaEnabled(false);
    mSwatch->setColor(mColour);
    mSwatch->setTooltip("Colour painted into the terrain colour map");

    mFill = mBar->createChild<rt::Button>("Fill");
    mFill->setTooltip("Plant the chosen species over the whole terrain, one tree per spacing cell "
                      "with a random offset, where the slope allows it");
    mClear = mBar->createChild<rt::Button>("Clear");
    mClear->setTooltip("Remove every tree of the forest");

    mStatus = mBar->createChild<rt::Label>(ct::String(kNoTerrainText));
    mStatus->setStretch(1.0f);

    const ct::Rc<Link> link = mLink;
    mModes->toggleChanged.connect(
            [link](int id, bool checked)
            {
                if (link->owner) link->owner->modeToggled(id, checked);
            });
    mLayerBox->selectionChanged.connect(
            [link](int index)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setLayer(index + 1);
            });
    mSpeciesBox->selectionChanged.connect(
            [link](int index)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setSpecies(index);
            });
    mRadiusField->valueChanged.connect(
            [link](float value)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setRadius(value);
            });
    mStrengthField->valueChanged.connect(
            [link](float value)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setStrength(value);
            });
    mSpacingField->valueChanged.connect(
            [link](float value)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setSpacing(value);
            });
    mSwatch->colorEdited.connect(
            [link](const Math::Vec4& colour)
            {
                if (link->owner && !link->owner->mSyncing) link->owner->setColour(colour);
            });
    mFill->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->fillForest();
            });
    mClear->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->clearForest();
            });
    syncBar();
    return true;
}

bool TerrainEditTool::attached() const
{
    return mBar != nullptr;
}

rt::Widget* TerrainEditTool::bar() const
{
    return mBar;
}

void TerrainEditTool::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
}

void TerrainEditTool::setSceneRoot(Node* root)
{
    mRoot = root;
}

void TerrainEditTool::setSelection(const EditorSelection* selection)
{
    mSelection = selection;
}

void TerrainEditTool::setActive(bool active)
{
    if (mActive == active) return;
    if (!active && mStroking) endStroke();
    mActive = active;
    mHovering = false;
    if (mBar) mBar->setVisible(active);
    if (active) refreshTarget();
}

bool TerrainEditTool::active() const
{
    return mActive;
}

void TerrainEditTool::setMode(Mode mode)
{
    if (mode < Mode::Raise || mode >= Mode::kCount) return;
    if (mStroking) endStroke();
    mMode = mode;
    syncBar();
}

TerrainEditTool::Mode TerrainEditTool::mode() const
{
    return mMode;
}

const char* TerrainEditTool::modeName(Mode mode)
{
    const int index = static_cast<int>(mode);
    return index >= 0 && index < static_cast<int>(Mode::kCount) ? kModeNames[index] : "";
}

TerrainEditTool::Kind TerrainEditTool::kindOf(Mode mode)
{
    switch (mode)
    {
        case Mode::Grass:
            return Kind::Grass;
        case Mode::Plant:
            return Kind::Forest;
        case Mode::Raise:
        case Mode::Smooth:
        case Mode::Flatten:
        case Mode::Paint:
        case Mode::Colour:
            return Kind::Terrain;
        default:
            return Kind::None;
    }
}

void TerrainEditTool::setRadius(float radius)
{
    if (radius < kMinRadius) radius = kMinRadius;
    if (radius > kMaxRadius) radius = kMaxRadius;
    mRadius = radius;
    syncBar();
}

float TerrainEditTool::radius() const
{
    return mRadius;
}

void TerrainEditTool::setStrength(float strength)
{
    if (strength < kMinStrength) strength = kMinStrength;
    if (strength > kMaxStrength) strength = kMaxStrength;
    mStrength = strength;
    syncBar();
}

float TerrainEditTool::strength() const
{
    return mStrength;
}

void TerrainEditTool::setSpacing(float spacing)
{
    if (spacing < kMinSpacing) spacing = kMinSpacing;
    if (spacing > kMaxSpacing) spacing = kMaxSpacing;
    mSpacing = spacing;
    syncBar();
}

float TerrainEditTool::spacing() const
{
    return mSpacing;
}

void TerrainEditTool::setLayer(int layer)
{
    if (layer < 1) layer = 1;
    if (mStroking) endStroke();
    mLayer = layer;
    syncBar();
}

int TerrainEditTool::layer() const
{
    return mLayer;
}

void TerrainEditTool::setSpecies(int species)
{
    if (species < 0) species = 0;
    if (mStroking) endStroke();
    mSpecies = species;
    syncBar();
}

int TerrainEditTool::species() const
{
    return mSpecies;
}

void TerrainEditTool::setColour(const Math::Vec4& colour)
{
    mColour = Math::Vec4(colour.x, colour.y, colour.z, 1.0f);
    if (mSwatch && !mSyncing) mSwatch->setColor(mColour);
}

const Math::Vec4& TerrainEditTool::colour() const
{
    return mColour;
}

TerrainEditTool::Kind TerrainEditTool::kind() const
{
    if (mGrass) return Kind::Grass;
    if (mForest) return Kind::Forest;
    return mTarget ? Kind::Terrain : Kind::None;
}

Terrain3D* TerrainEditTool::target() const
{
    return mTarget;
}

Grass3D* TerrainEditTool::grass() const
{
    return mGrass;
}

Forest3D* TerrainEditTool::forest() const
{
    return mForest;
}

bool TerrainEditTool::stroking() const
{
    return mStroking;
}

bool TerrainEditTool::hovering() const
{
    return mHovering;
}

const Vec3& TerrainEditTool::hitPosition() const
{
    return mHit;
}

unsigned int TerrainEditTool::strokeCount() const
{
    return mStrokes;
}

void TerrainEditTool::refreshTarget()
{
    Terrain3D* terrain = nullptr;
    Grass3D* grass = nullptr;
    Forest3D* forest = nullptr;
    if (mSelection)
    {
        const ct::Vector<Node*>& nodes = mSelection->selectedNodes();
        for (std::size_t i = 0; i < nodes.size() && !terrain && !grass && !forest; ++i)
        {
            for (Node* node = nodes[i]; node; node = node->getParent())
            {
                if ((grass = node->castTo<Grass3D>())) break;
                if ((forest = node->castTo<Forest3D>())) break;
                if ((terrain = node->castTo<Terrain3D>())) break;
            }
        }
    }
    if (grass) terrain = grass->getTerrain();
    if (forest) terrain = forest->getTerrain();
    if (!terrain && mRoot)
    {
        ct::Vector<Node*> stack;
        stack.push_back(mRoot);
        while (!stack.empty() && !terrain)
        {
            Node* node = stack.back();
            stack.pop_back();
            terrain = node->castTo<Terrain3D>();
            for (std::size_t i = 0; i < node->getChildCount(); ++i)
                stack.push_back(node->getChild(i));
        }
    }
    if (terrain && !terrain->isLoaded()) terrain = nullptr;

    const bool changed = terrain != mTarget || grass != mGrass || forest != mForest;
    if (changed)
    {
        if (mStroking) endStroke();
        mTarget = terrain;
        mGrass = grass;
        mForest = forest;
        mLayerCount = -1;
        mSpeciesCount = -1;
        const Kind wanted = kind();
        if (wanted == Kind::Grass && mMode != Mode::Grass) mMode = Mode::Grass;
        else if (wanted == Kind::Forest && mMode != Mode::Plant)
            mMode = Mode::Plant;
        else if (wanted == Kind::Terrain && kindOf(mMode) != Kind::Terrain)
            mMode = Mode::Raise;
    }
    syncLayers();
    syncSpecies();
    if (changed) syncBar();
}

void TerrainEditTool::syncLayers()
{
    const int count = mTarget ? mTarget->getLayerCount() : 0;
    if (count == mLayerCount) return;
    mLayerCount = count;
    if (!mLayerBox) return;
    mSyncing = true;
    mLayerBox->clear();
    for (int i = 1; i < count; ++i)
    {
        ct::String text("Layer ");
        text.append_number(static_cast<unsigned long long>(i + 1));
        mLayerBox->addItem(text);
    }
    if (mLayer > count - 1) mLayer = count > 1 ? count - 1 : 1;
    mLayerBox->setSelectedIndex(count > 1 ? mLayer - 1 : -1);
    mSyncing = false;
}

void TerrainEditTool::syncSpecies()
{
    const int count = mForest ? mForest->getSpeciesCount() : 0;
    if (count == mSpeciesCount) return;
    mSpeciesCount = count;
    if (!mSpeciesBox) return;
    mSyncing = true;
    mSpeciesBox->clear();
    for (int i = 0; i < count; ++i)
    {
        ct::String text("Species ");
        text.append_number(static_cast<unsigned long long>(i + 1));
        mSpeciesBox->addItem(text);
    }
    if (mSpecies >= count) mSpecies = count > 0 ? count - 1 : 0;
    mSpeciesBox->setSelectedIndex(count > 0 ? mSpecies : -1);
    mSyncing = false;
}

void TerrainEditTool::syncBar()
{
    if (!mBar) return;
    mSyncing = true;
    const Kind current = kind();
    for (int i = 0; i < static_cast<int>(Mode::kCount); ++i)
    {
        const Mode mode = static_cast<Mode>(i);
        const bool checked = mode == mMode;
        if (mModes->isItemChecked(mModeButtons[i]) != checked)
            mModes->setItemChecked(mModeButtons[i], checked);
        const Kind owner = kindOf(mode);
        const bool shown = current == Kind::None || owner == current ||
                           (current == Kind::Terrain && owner == Kind::Terrain);
        mModes->setItemVisible(mModeButtons[i], shown);
    }
    const bool paint = mMode == Mode::Paint;
    const bool plant = mMode == Mode::Plant;
    mSwatch->setVisible(mMode == Mode::Colour);
    mLayerLabel->setVisible(paint);
    mLayerBox->setVisible(paint);
    mSpeciesLabel->setVisible(plant);
    mSpeciesBox->setVisible(plant);
    mSpacingLabel->setVisible(plant);
    mSpacingField->setVisible(plant);
    mFill->setVisible(plant);
    mClear->setVisible(plant);
    mFill->setEnabled(mForest && mTarget && mSpeciesCount > 0);
    mClear->setEnabled(mForest && mForest->getInstanceCount() > 0);
    if (mLayerCount > 1 && mLayerBox->selectedIndex() != mLayer - 1)
        mLayerBox->setSelectedIndex(mLayer - 1);
    if (mSpeciesCount > 0 && mSpeciesBox->selectedIndex() != mSpecies)
        mSpeciesBox->setSelectedIndex(mSpecies);
    mRadiusField->setValue(mRadius);
    mStrengthField->setValue(mStrength);
    mSpacingField->setValue(mSpacing);

    ct::String status;
    if (mGrass && !mTarget) status = ct::String("Set a terrain on ") + mGrass->getName();
    else if (mForest && !mTarget)
        status = ct::String("Set a terrain on ") + mForest->getName();
    else if (!mTarget)
        status = kNoTerrainText;
    else if (mGrass && !mGrass->getTexture().valid())
        status = ct::String("Set a texture on ") + mGrass->getName();
    else if (mGrass)
        status = ct::String("Painting ") + mGrass->getName();
    else if (mForest && mSpeciesCount < 1)
        status = ct::String("Add a species to ") + mForest->getName();
    else if (mForest)
        status = ct::String("Planting ") + mForest->getName();
    else if (paint && mLayerCount < 2)
        status = "Add a second layer to paint";
    else
        status = ct::String("Editing ") + mTarget->getName();
    mStatus->setText(status);
    mSyncing = false;
}

void TerrainEditTool::modeToggled(int id, bool checked)
{
    if (mSyncing) return;
    for (int i = 0; i < static_cast<int>(Mode::kCount); ++i)
    {
        if (mModeButtons[i] != id) continue;
        if (checked) setMode(static_cast<Mode>(i));
        else
            syncBar();
        return;
    }
}

bool TerrainEditTool::computeHit(const EditorView& view)
{
    if (!mTarget) return false;
    const Math::Ray ray(view.rayOrigin(mMouseX, mMouseY), view.rayDirection(mMouseX, mMouseY));
    Vec3 hit;
    if (!mTarget->rayIntersects(ray, hit)) return false;
    mHit = hit;
    return true;
}

bool TerrainEditTool::handleEvent(const InputEvent& event, float x, float y, bool navigating)
{
    if (!mActive) return false;
    switch (event.type)
    {
        case InputEventType::Key:
        {
            const Key key = event.key.key;
            if (key == Key::LeftShift || key == Key::RightShift) mInvert = event.key.pressed;
            return false;
        }
        case InputEventType::MouseMotion:
            mMouseX = x;
            mMouseY = y;
            return mStroking;
        case InputEventType::MouseButton:
            mMouseX = x;
            mMouseY = y;
            mInvert = (event.mouseButton.modifiers & kModShift) != 0;
            if (event.mouseButton.button == MouseButton::Left)
            {
                if (event.mouseButton.pressed)
                {
                    if (navigating || !mTarget || !mHovering) return false;
                    beginStroke();
                    return mStroking;
                }
                if (!mStroking) return false;
                endStroke();
                return true;
            }
            if (event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                    mStroking)
            {
                cancelStroke();
                return true;
            }
            return false;
        default:
            return false;
    }
}

void TerrainEditTool::update(float delta, const EditorView& view)
{
    if (!mActive) return;
    refreshTarget();
    mHovering = computeHit(view);
    if (!mStroking) return;
    if (!mTarget)
    {
        mStroking = false;
        return;
    }
    if (mHovering) applyBrush(delta);
    mSinceUpdate += delta;
    if (mStrokeMoved && mSinceUpdate >= kUpdateInterval && kindOf(mMode) == Kind::Terrain)
    {
        mTarget->update();
        mSinceUpdate = 0.0f;
    }
}

const float* TerrainEditTool::currentData(std::size_t& size) const
{
    size = 0;
    if (!mTarget) return nullptr;
    Terrain& terrain = const_cast<Terrain3D*>(mTarget)->getTerrain();
    if (mMode == Mode::Paint)
    {
        TerrainLayerBlendMap* map = terrain.getLayerBlendMap(static_cast<std::uint8_t>(mLayer));
        if (!map) return nullptr;
        size = static_cast<std::size_t>(map->getSize()) * map->getSize();
        return map->getBlendPointer();
    }
    size = static_cast<std::size_t>(terrain.getSize()) * terrain.getSize();
    return terrain.getHeightData();
}

void TerrainEditTool::takeSnapshot()
{
    std::size_t size = 0;
    const float* data = currentData(size);
    mSnapshot.resize(size);
    for (std::size_t i = 0; i < size; ++i) mSnapshot[i] = data[i];
}

bool TerrainEditTool::diffSnapshot(Block& block) const
{
    std::size_t size = 0;
    const float* data = currentData(size);
    if (!data || size != mSnapshot.size() || size == 0) return false;
    const int side = static_cast<int>(sqrt(static_cast<double>(size)) + 0.5);
    int left = side, top = side, right = -1, bottom = -1;
    for (int y = 0; y < side; ++y)
    {
        for (int x = 0; x < side; ++x)
        {
            const std::size_t i = static_cast<std::size_t>(y) * side + x;
            if (data[i] == mSnapshot[i]) continue;
            if (x < left) left = x;
            if (x > right) right = x;
            if (y < top) top = y;
            if (y > bottom) bottom = y;
        }
    }
    if (right < left) return false;
    block.rect = TerrainRect(left, top, right + 1, bottom + 1);
    const int width = block.rect.width();
    const int height = block.rect.height();
    block.before.resize(static_cast<std::size_t>(width) * height);
    block.after.resize(block.before.size());
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const std::size_t src = static_cast<std::size_t>(top + y) * side + (left + x);
            const std::size_t dst = static_cast<std::size_t>(y) * width + x;
            block.before[dst] = mSnapshot[src];
            block.after[dst] = data[src];
        }
    }
    return true;
}

void TerrainEditTool::applyBlock(Terrain3D* terrain, Mode mode, int layer, const TerrainRect& rect,
        const ct::Vector<float>& values)
{
    if (!terrain || !terrain->isLoaded()) return;
    Terrain& data = terrain->getTerrain();
    const int width = rect.width();
    const int height = rect.height();
    if (mode == Mode::Paint)
    {
        TerrainLayerBlendMap* map = data.getLayerBlendMap(static_cast<std::uint8_t>(layer));
        if (!map) return;
        const int side = map->getSize();
        float* pixels = map->getBlendPointer();
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                pixels[static_cast<std::size_t>(rect.top + y) * side + (rect.left + x)] =
                        values[static_cast<std::size_t>(y) * width + x];
        map->dirtyRect(rect);
        map->update();
        terrain->update();
        return;
    }
    const int side = data.getSize();
    float* heights = data.getHeightData();
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            heights[static_cast<std::size_t>(rect.top + y) * side + (rect.left + x)] =
                    values[static_cast<std::size_t>(y) * width + x];
    data.dirtyRect(rect);
    terrain->update();
}

void TerrainEditTool::applyMask(Grass3D* grass, const ct::Vector<std::uint8_t>& pixels)
{
    if (!grass) return;
    if (!grass->hasMask()) grass->createMask();
    grass->setMaskPixels(pixels);
}

void TerrainEditTool::snapshotInstances(const Forest3D* forest, ct::Vector<InstanceRecord>& out)
{
    out.clear();
    if (!forest) return;
    const int count = forest->getInstanceCount();
    out.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        out[static_cast<std::size_t>(i)].species = forest->getInstanceSpecies(i);
        out[static_cast<std::size_t>(i)].transform = forest->getInstanceTransform(i);
        out[static_cast<std::size_t>(i)].onTerrain = forest->isInstanceOnTerrain(i);
    }
}

void TerrainEditTool::applyInstances(Forest3D* forest, const ct::Vector<InstanceRecord>& records)
{
    if (!forest) return;
    forest->clearInstances();
    for (std::size_t i = 0; i < records.size(); ++i)
        forest->addInstance(records[i].species, records[i].transform, records[i].onTerrain);
}

void TerrainEditTool::ensureGrassExtent()
{
    if (!mGrass || !mTarget || mGrass->hasExtent()) return;
    const Math::Box box = mTarget->getAabb();
    const Math::Mat4 inverse = mGrass->getGlobalTransform().InverseAffine();
    const Vec3 corners[4] = { Vec3(box.min.x, 0.0f, box.min.z), Vec3(box.max.x, 0.0f, box.min.z),
        Vec3(box.min.x, 0.0f, box.max.z), Vec3(box.max.x, 0.0f, box.max.z) };
    float minX = 0.0f, minZ = 0.0f, maxX = 0.0f, maxZ = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        const Vec3 local = inverse.TransformPoint(corners[i]);
        if (i == 0 || local.x < minX) minX = local.x;
        if (i == 0 || local.x > maxX) maxX = local.x;
        if (i == 0 || local.z < minZ) minZ = local.z;
        if (i == 0 || local.z > maxZ) maxZ = local.z;
    }
    mGrass->setExtent(minX, minZ, maxX, maxZ);
}

float TerrainEditTool::nextRandom()
{
    mRandom = mRandom * 1664525u + 1013904223u;
    return static_cast<float>(mRandom >> 8) / 16777216.0f;
}

void TerrainEditTool::beginStroke()
{
    if (!mTarget || mStroking) return;
    switch (kindOf(mMode))
    {
        case Kind::Terrain:
            if (mMode == Mode::Paint && mLayerCount < 2) return;
            if (mMode == Mode::Colour)
            {
                if (!mTarget->getColourMapEnabled()) mTarget->setColourMapEnabled(true);
                if (!mTarget->getTerrain().globalColourMapData()) return;
                mColourSnapshot = mTarget->getTerrain().getGlobalColourMapData();
                break;
            }
            takeSnapshot();
            if (mSnapshot.empty()) return;
            mFlattenHeight = mTarget->getTerrain().getHeightAtWorldPosition(mHit);
            break;
        case Kind::Grass:
            if (!mGrass || !mGrass->getTexture().valid()) return;
            ensureGrassExtent();
            if (!mGrass->hasMask() && !mGrass->createMask()) return;
            if (!mGrass->getMaskPixels(mMaskSnapshot)) return;
            break;
        case Kind::Forest:
            if (!mForest || mSpeciesCount < 1) return;
            snapshotInstances(mForest, mInstanceSnapshot);
            mPlantAccumulator = 1.0f;
            break;
        default:
            return;
    }
    mStroking = true;
    mStrokeMoved = false;
    mSinceUpdate = 0.0f;
}

void TerrainEditTool::applyBrush(float delta)
{
    switch (kindOf(mMode))
    {
        case Kind::Grass:
            applyGrass(delta);
            return;
        case Kind::Forest:
            applyPlant(delta);
            return;
        default:
            break;
    }
    Terrain& terrain = mTarget->getTerrain();
    bool changed = false;
    switch (mMode)
    {
        case Mode::Raise:
            changed = terrainBrushHeight(terrain, mHit, mRadius, kHeightStrength * mStrength, delta,
                    mInvert);
            break;
        case Mode::Smooth:
            changed = terrainBrushSmooth(terrain, mHit, mRadius, mStrength, delta * 4.0f);
            break;
        case Mode::Flatten:
            changed = terrainBrushFlatten(terrain, mHit, mRadius, mFlattenHeight, mStrength,
                    delta * 4.0f);
            break;
        case Mode::Paint:
            changed = terrainBrushBlend(terrain, static_cast<std::uint8_t>(mLayer), mHit, mRadius,
                    kBlendStrength * mStrength, delta, mInvert);
            break;
        case Mode::Colour:
            applyColour(delta);
            return;
        default:
            break;
    }
    if (changed) mStrokeMoved = true;
}

void TerrainEditTool::applyColour(float delta)
{
    if (terrainBrushColour(mTarget->getTerrain(), mHit, mRadius,
                Math::Vec3(mColour.x, mColour.y, mColour.z), kColourStrength * mStrength, delta))
        mStrokeMoved = true;
}

bool TerrainEditTool::diffColour(TerrainRect& rect, ct::Vector<std::uint8_t>& before,
        ct::Vector<std::uint8_t>& after) const
{
    const ct::Vector<std::uint8_t>& current = mTarget->getTerrain().getGlobalColourMapData();
    if (current.size() != mColourSnapshot.size() || current.empty()) return false;
    const int side = mTarget->getTerrain().getGlobalColourMapSize();
    int left = side, top = side, right = -1, bottom = -1;
    for (int y = 0; y < side; ++y)
    {
        for (int x = 0; x < side; ++x)
        {
            const std::size_t i = (static_cast<std::size_t>(y) * side + x) * 3;
            if (current[i] == mColourSnapshot[i] && current[i + 1] == mColourSnapshot[i + 1] &&
                    current[i + 2] == mColourSnapshot[i + 2])
                continue;
            if (x < left) left = x;
            if (x > right) right = x;
            if (y < top) top = y;
            if (y > bottom) bottom = y;
        }
    }
    if (right < left) return false;
    rect = TerrainRect(left, top, right + 1, bottom + 1);
    const std::size_t width = static_cast<std::size_t>(rect.width());
    before.resize(width * rect.height() * 3);
    after.resize(before.size());
    for (int y = 0; y < rect.height(); ++y)
    {
        const std::size_t src = (static_cast<std::size_t>(top + y) * side + left) * 3;
        const std::size_t dst = static_cast<std::size_t>(y) * width * 3;
        for (std::size_t i = 0; i < width * 3; ++i)
        {
            before[dst + i] = mColourSnapshot[src + i];
            after[dst + i] = current[src + i];
        }
    }
    return true;
}

void TerrainEditTool::applyColourBlock(Terrain3D* terrain, const TerrainRect& rect,
        const ct::Vector<std::uint8_t>& values)
{
    if (!terrain || !terrain->isLoaded()) return;
    if (!terrain->getColourMapEnabled()) terrain->setColourMapEnabled(true);
    Terrain& data = terrain->getTerrain();
    std::uint8_t* pixels = data.globalColourMapData();
    if (!pixels) return;
    const int side = data.getGlobalColourMapSize();
    const std::size_t width = static_cast<std::size_t>(rect.width());
    for (int y = 0; y < rect.height(); ++y)
    {
        if (rect.top + y >= side) break;
        std::uint8_t* row =
                &pixels[(static_cast<std::size_t>(rect.top + y) * side + rect.left) * 3];
        const std::uint8_t* source = &values[static_cast<std::size_t>(y) * width * 3];
        for (std::size_t i = 0; i < width * 3 && rect.left + static_cast<int>(i / 3) < side; ++i)
            row[i] = source[i];
    }
    data.dirtyGlobalColourMapRect(rect);
    terrain->update();
    terrain->flushCompositeMap();
}

void TerrainEditTool::applyGrass(float delta)
{
    if (!mGrass) return;
    const float amount = kGrassStrength * mStrength * delta;
    const bool changed = mInvert ? mGrass->erase(mHit, mRadius, amount)
                                 : mGrass->paint(mHit, mRadius, amount, 0);
    if (changed) mStrokeMoved = true;
}

void TerrainEditTool::applyPlant(float delta)
{
    if (!mForest) return;
    if (mInvert)
    {
        if (mForest->removeInstancesInRadius(mHit, mRadius) > 0) mStrokeMoved = true;
        return;
    }
    mPlantAccumulator += kPlantRate * mStrength * delta;
    const Math::Mat4 world = mForest->getGlobalTransform();
    const Math::Mat4 inverse = world.InverseAffine();
    const bool onTerrain = mForest->getTerrain() == mTarget;
    while (mPlantAccumulator >= 1.0f)
    {
        mPlantAccumulator -= 1.0f;
        for (int attempt = 0; attempt < kPlantAttempts; ++attempt)
        {
            const float distance = mRadius * sqrtf(nextRandom());
            const float angle = 2.0f * kPi * nextRandom();
            const Vec3 flat(mHit.x + cosf(angle) * distance, 0.0f, mHit.z + sinf(angle) * distance);
            bool blocked = false;
            if (mSpacing > 0.0f)
            {
                const int count = mForest->getInstanceCount();
                for (int i = 0; i < count && !blocked; ++i)
                {
                    const Vec3 other =
                            world.TransformPoint(mForest->getInstanceTransform(i).GetTranslation());
                    const float dx = other.x - flat.x;
                    const float dz = other.z - flat.z;
                    blocked = dx * dx + dz * dz < mSpacing * mSpacing;
                }
            }
            if (blocked) continue;
            const float yaw = 2.0f * kPi * nextRandom();
            const float scale = 0.8f + 0.4f * nextRandom();
            int planted = -1;
            if (onTerrain)
            {
                const Vec3 local = inverse.TransformPoint(flat);
                planted = mForest->plantOnTerrain(mSpecies, local.x, local.z, yaw, scale);
            }
            else
            {
                const Vec3 ground(flat.x, mTarget->getWorldHeightAt(flat), flat.z);
                planted = mForest->plant(mSpecies, inverse.TransformPoint(ground), yaw, scale);
            }
            if (planted >= 0)
            {
                mStrokeMoved = true;
                break;
            }
        }
    }
}

void TerrainEditTool::recordInstances(const char* action, ct::Vector<InstanceRecord>& before)
{
    ++mStrokes;
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (!undo)
    {
        if (mListener) mListener->sceneEdited();
        return;
    }
    ct::Rc<PlantEdit> edit = ct::make_rc<PlantEdit>();
    edit->before = static_cast<ct::Vector<InstanceRecord>&&>(before);
    snapshotInstances(mForest, edit->after);
    Forest3D* forest = mForest;
    undo->createAction(ct::String(action));
    undo->addDoMethod([forest, edit] { applyInstances(forest, edit->after); }, forest);
    undo->addUndoMethod([forest, edit] { applyInstances(forest, edit->before); }, forest);
    undo->commitAction(false);
}

void TerrainEditTool::fillForest()
{
    if (!mForest || !mTarget || mSpeciesCount < 1) return;
    if (mStroking) endStroke();
    const float step = mSpacing > kMinRadius ? mSpacing : kMinRadius;
    const Math::Box box = mTarget->getAabb();
    const float width = box.max.x - box.min.x;
    const float depth = box.max.z - box.min.z;
    if (width <= step || depth <= step) return;
    const int columns = static_cast<int>(width / step);
    const int rows = static_cast<int>(depth / step);
    if (static_cast<long long>(columns) * rows > 4000000ll) return;

    ct::Vector<InstanceRecord> before;
    snapshotInstances(mForest, before);
    const Math::Mat4 world = mForest->getGlobalTransform();
    const Math::Mat4 inverse = world.InverseAffine();
    const bool onTerrain = mForest->getTerrain() == mTarget;
    int planted = 0;
    for (int row = 0; row < rows; ++row)
    {
        for (int column = 0; column < columns; ++column)
        {
            const float x =
                    box.min.x + (static_cast<float>(column) + 0.25f + 0.5f * nextRandom()) * step;
            const float z =
                    box.min.z + (static_cast<float>(row) + 0.25f + 0.5f * nextRandom()) * step;
            const Vec3 flat(x, 0.0f, z);
            const float yaw = 2.0f * kPi * nextRandom();
            const float scale = 0.8f + 0.4f * nextRandom();
            int index = -1;
            if (onTerrain)
            {
                const Vec3 local = inverse.TransformPoint(flat);
                index = mForest->plantOnTerrain(mSpecies, local.x, local.z, yaw, scale);
            }
            else
            {
                const Vec3 ground(flat.x, mTarget->getWorldHeightAt(flat), flat.z);
                index = mForest->plant(mSpecies, inverse.TransformPoint(ground), yaw, scale);
            }
            if (index >= 0) ++planted;
        }
    }
    if (planted == 0) return;
    recordInstances(kFillActionName, before);
    syncBar();
}

void TerrainEditTool::clearForest()
{
    if (!mForest || mForest->getInstanceCount() == 0) return;
    if (mStroking) endStroke();
    ct::Vector<InstanceRecord> before;
    snapshotInstances(mForest, before);
    mForest->clearInstances();
    recordInstances(kClearActionName, before);
    syncBar();
}

rt::Button* TerrainEditTool::fillButton() const
{
    return mFill;
}

rt::Button* TerrainEditTool::clearButton() const
{
    return mClear;
}

void TerrainEditTool::endStroke()
{
    if (!mStroking) return;
    mStroking = false;
    if (!mTarget) return;
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;

    if (kindOf(mMode) == Kind::Grass)
    {
        if (!mGrass || !mStrokeMoved) return;
        ct::Rc<MaskEdit> edit = ct::make_rc<MaskEdit>();
        edit->before = static_cast<ct::Vector<std::uint8_t>&&>(mMaskSnapshot);
        if (!mGrass->getMaskPixels(edit->after) || sameBytes(edit->before, edit->after)) return;
        ++mStrokes;
        if (!undo)
        {
            if (mListener) mListener->sceneEdited();
            return;
        }
        Grass3D* grass = mGrass;
        undo->createAction(ct::String(kGrassActionName));
        undo->addDoMethod([grass, edit] { applyMask(grass, edit->after); }, grass);
        undo->addUndoMethod([grass, edit] { applyMask(grass, edit->before); }, grass);
        undo->commitAction(false);
        return;
    }

    if (kindOf(mMode) == Kind::Forest)
    {
        if (!mForest || !mStrokeMoved) return;
        ct::Rc<PlantEdit> edit = ct::make_rc<PlantEdit>();
        edit->before = static_cast<ct::Vector<InstanceRecord>&&>(mInstanceSnapshot);
        snapshotInstances(mForest, edit->after);
        ++mStrokes;
        if (!undo)
        {
            if (mListener) mListener->sceneEdited();
            return;
        }
        Forest3D* forest = mForest;
        undo->createAction(ct::String(kPlantActionName));
        undo->addDoMethod([forest, edit] { applyInstances(forest, edit->after); }, forest);
        undo->addUndoMethod([forest, edit] { applyInstances(forest, edit->before); }, forest);
        undo->commitAction(false);
        return;
    }

    if (mMode == Mode::Colour)
    {
        mTarget->update();
        mTarget->flushCompositeMap();
        if (!mStrokeMoved) return;
        ct::Rc<ColourEdit> edit = ct::make_rc<ColourEdit>();
        if (!diffColour(edit->rect, edit->before, edit->after)) return;
        ++mStrokes;
        if (!undo)
        {
            if (mListener) mListener->sceneEdited();
            return;
        }
        Terrain3D* terrain = mTarget;
        undo->createAction(ct::String(kColourActionName));
        undo->addDoMethod([terrain, edit] { applyColourBlock(terrain, edit->rect, edit->after); },
                terrain);
        undo->addUndoMethod([terrain, edit]
                { applyColourBlock(terrain, edit->rect, edit->before); }, terrain);
        undo->commitAction(false);
        return;
    }

    mTarget->update();
    ct::Rc<BlockEdit> edit = ct::make_rc<BlockEdit>();
    edit->mode = mMode;
    edit->layer = mLayer;
    Block block;
    if (!diffSnapshot(block)) return;
    edit->rect = block.rect;
    edit->before = static_cast<ct::Vector<float>&&>(block.before);
    edit->after = static_cast<ct::Vector<float>&&>(block.after);
    ++mStrokes;
    if (!undo)
    {
        if (mListener) mListener->sceneEdited();
        return;
    }
    Terrain3D* terrain = mTarget;
    undo->createAction(ct::String(kActionName));
    undo->addDoMethod([terrain, edit]
            { applyBlock(terrain, edit->mode, edit->layer, edit->rect, edit->after); }, terrain);
    undo->addUndoMethod([terrain, edit]
            { applyBlock(terrain, edit->mode, edit->layer, edit->rect, edit->before); }, terrain);
    undo->commitAction(false);
}

void TerrainEditTool::cancelStroke()
{
    if (!mStroking) return;
    mStroking = false;
    if (!mTarget) return;
    switch (kindOf(mMode))
    {
        case Kind::Grass:
            if (mGrass && mStrokeMoved) applyMask(mGrass, mMaskSnapshot);
            return;
        case Kind::Forest:
            if (mForest && mStrokeMoved) applyInstances(mForest, mInstanceSnapshot);
            return;
        default:
            break;
    }
    if (mMode == Mode::Colour)
    {
        TerrainRect rect;
        ct::Vector<std::uint8_t> before;
        ct::Vector<std::uint8_t> after;
        if (mStrokeMoved && diffColour(rect, before, after))
            applyColourBlock(mTarget, rect, before);
        return;
    }
    Block block;
    if (diffSnapshot(block)) applyBlock(mTarget, mMode, mLayer, block.rect, block.before);
}

void TerrainEditTool::draw(BatchRenderer& batch)
{
    if (!mActive || !mTarget || !mHovering) return;
    const BatchRenderer::Space space = batch.space();
    batch.setSpace(BatchRenderer::Space::World);
    batch.pushMatrix();
    batch.loadIdentity();
    batch.setTexture(batch.whiteTexture());
    batch.setDepthTest(false);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setCullFace(false);

    const float* color = kModeColors[static_cast<int>(mMode)];
    const bool inverted =
            mInvert && mMode != Mode::Smooth && mMode != Mode::Flatten && mMode != Mode::Colour;
    if (mMode == Mode::Colour)
        batch.setColor(mColour.x, mColour.y, mColour.z, mStroking ? 1.0f : 0.8f);
    else
        batch.setColor(inverted ? 1.0f : color[0], inverted ? 0.35f : color[1],
                inverted ? 0.35f : color[2], mStroking ? 1.0f : 0.8f);
    batch.begin(BatchRenderer::ModeLines);
    Vec3 previous;
    for (int i = 0; i <= kRingSegments; ++i)
    {
        const float angle = 2.0f * kPi * static_cast<float>(i) / kRingSegments;
        Vec3 point(mHit.x + cosf(angle) * mRadius, 0.0f, mHit.z + sinf(angle) * mRadius);
        point.y = mTarget->getWorldHeightAt(point) + kRingLift;
        if (i > 0)
        {
            batch.vertex3(previous.x, previous.y, previous.z);
            batch.vertex3(point.x, point.y, point.z);
        }
        previous = point;
    }
    const float cross = mRadius * 0.1f;
    batch.vertex3(mHit.x - cross, mHit.y + kRingLift, mHit.z);
    batch.vertex3(mHit.x + cross, mHit.y + kRingLift, mHit.z);
    batch.vertex3(mHit.x, mHit.y + kRingLift, mHit.z - cross);
    batch.vertex3(mHit.x, mHit.y + kRingLift, mHit.z + cross);
    batch.end();

    batch.popMatrix();
    batch.setSpace(space);
}

} // namespace prisma3d
