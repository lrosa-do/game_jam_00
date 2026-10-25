#include "prisma3d/editor/LightmapPanel.h"

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/LightmapMeshIO.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/scene/SceneTree.h"

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <stdio.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kPanelPadding = 8.0f;
const float kSpacing = 4.0f;
const float kLabelWidth = 130.0f;

struct FieldDef
{
    const char* label;
    const char* tip;
    double minimum;
    double maximum;
    float sensitivity;
};

const FieldDef kFields[LightmapPanel::kFieldCount] = {
    { "Resolution", "Size in texels of the lightmap of each mesh (64 to 4096).", 64.0, 4096.0,
        8.0f },
    { "Padding",
        "Texels of empty space around each chart of the UV2 unwrap, so bilinear "
        "filtering cannot pick up a neighbour.",
        0.0, 32.0, 0.1f },
    { "Shadow resolution",
        "Size of the sun's depth map. It limits how sharp a baked shadow "
        "edge can be.",
        256.0, 16384.0, 64.0f },
    { "Samples", "Shadow maps averaged over the sun's disk to give a penumbra.", 1.0, 64.0, 0.1f },
    { "Sun angular radius",
        "Radius of the sun in degrees. Zero gives hard shadows. The "
        "light's own angular distance is used when it is larger.",
        0.0, 10.0, 0.02f },
    { "Filter radius", "Shadow map PCF radius in texels. Zero is a single hard compare.", 0.0, 16.0,
        0.05f },
    { "Bias texels", "Depth bias measured in shadow map texels.", 0.0, 32.0, 0.05f },
    { "Ambient ground", "Ground bounce as a fraction of the sky ambient.", 0.0, 1.0, 0.005f }
};

std::uint64_t mix(std::uint64_t hash, std::uint64_t value)
{
    return (hash ^ value) * 1099511628211ull;
}

std::uint64_t hashString(std::uint64_t hash, const ct::String& text)
{
    for (std::size_t i = 0; i < text.size(); ++i)
        hash = mix(hash, static_cast<unsigned char>(text.data()[i]));
    return mix(hash, text.size());
}

void requestLayout()
{
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

ct::String colorText(const Math::Vec3& color)
{
    char text[96];
    snprintf(text, sizeof(text), "%.3f, %.3f, %.3f", static_cast<double>(color.x),
            static_cast<double>(color.y), static_cast<double>(color.z));
    return ct::String(text);
}

} // namespace

LightmapPanel::LightmapPanel(LightmapBaker& baker, EditorDocument& document,
        EditorSelection& selection)
        : mBaker(baker),
          mDocument(document),
          mSelection(selection),
          mLink(ct::make_rc<Link>(Link{ this })),
          mScroll(nullptr),
          mRoot(nullptr),
          mNodeLabel(nullptr),
          mDataLabel(nullptr),
          mNodeButton(nullptr),
          mSunLabel(nullptr),
          mSunButton(nullptr),
          mAmbientLabel(nullptr),
          mMeshesLabel(nullptr),
          mRows(nullptr),
          mStaticButton(nullptr),
          mDraft(nullptr),
          mFinal(nullptr),
          mFields{},
          mUnwrap(nullptr),
          mBake(nullptr),
          mCancel(nullptr),
          mProgress(nullptr),
          mStatus(nullptr),
          mSummary(nullptr),
          mSignature(0),
          mSyncing(false)
{
}

LightmapPanel::~LightmapPanel()
{
    mLink->owner = nullptr;
}

bool LightmapPanel::attach(rt::Widget* parent)
{
    if (mRoot || !parent) return false;
    const ct::Rc<Link> link = mLink;
    const rt::Theme& theme = rt::Theme::instance();

    mScroll = parent->createChild<rt::ScrollView>();
    mRoot = mScroll->setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRoot->setPadding(kPanelPadding);
    mRoot->setSpacing(kSpacing * 2.0f);

    mRoot->createChild<CategoryHeader>(ct::String("Scene"), EditorIcons::get("Light3D"));
    mNodeLabel = mRoot->createChild<rt::Label>(ct::String(kNoNodeText));
    mDataLabel = mRoot->createChild<rt::Label>(ct::String());
    mDataLabel->setColor(theme.textDisabled);
    mNodeButton = mRoot->createChild<rt::Button>(ct::String("Add LightmapGI"));
    mNodeButton->setTooltip(
            "Create the LightmapGI node that holds the baked lightmap, or select it");
    mNodeButton->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->createNode();
            });

    mSunLabel = mRoot->createChild<rt::Label>(ct::String(kNoSunText));
    mSunButton = mRoot->createChild<rt::Button>(ct::String("Make sun Static"));
    mSunButton->setTooltip("Set the Bake Mode of the sun to Static so the baked light replaces "
                           "its direct light on lightmapped meshes");
    mSunButton->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->setSunStatic();
            });
    mAmbientLabel = mRoot->createChild<rt::Label>(ct::String());
    mAmbientLabel->setColor(theme.textDisabled);

    mRoot->createChild<CategoryHeader>(ct::String("Bake"), EditorIcons::get("Play"));
    mUnwrap = mRoot->createChild<rt::Button>(ct::String("Unwrap UV2"));
    mUnwrap->setIconId(EditorIcons::get("Reload"));
    mUnwrap->setTooltip("Generate the second UV set of every Static mesh that has none");
    mUnwrap->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->unwrapMissing();
            });
    rt::BoxLayout* actions = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    actions->setPadding(0.0f);
    actions->setSpacing(kSpacing);
    mBake = actions->createChild<rt::Button>(ct::String("Bake"));
    mBake->setStretch(1.0f);
    mBake->setIconId(EditorIcons::get("Play"));
    mBake->setTooltip("Bake the Static sun and the ambient of the scene into the lightmap of every "
                      "Static mesh, unwrapping first the meshes that need it");
    mBake->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->bake();
            });
    mCancel = actions->createChild<rt::Button>(ct::String("Cancel"));
    mCancel->setStretch(1.0f);
    mCancel->setIconId(EditorIcons::get("Close"));
    mCancel->setTooltip("Stop the running job. Nothing is written until the whole bake ends.");
    mCancel->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->cancel();
            });

    mProgress = mRoot->createChild<rt::ProgressBar>(0.0f, 1.0f, 0.0f);
    mStatus = mRoot->createChild<rt::Label>(ct::String());
    mSummary = mRoot->createChild<rt::Label>(ct::String());
    mSummary->setColor(theme.textDisabled);
    mRoot->createChild<CategoryHeader>(ct::String("Meshes"), EditorIcons::get("MeshInstance3D"));
    mMeshesLabel = mRoot->createChild<rt::Label>(ct::String());
    mMeshesLabel->setColor(theme.textDisabled);
    mRows = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRows->setPadding(0.0f);
    mRows->setSpacing(2.0f);
    mStaticButton = mRoot->createChild<rt::Button>(ct::String("Mark selected Static"));
    mStaticButton->setTooltip("Set the GI Mode of the selected meshes to Static");
    mStaticButton->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->markSelectedStatic();
            });

    mRoot->createChild<CategoryHeader>(ct::String("Settings"), EditorIcons::get("Tools"));
    rt::BoxLayout* presets = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    presets->setPadding(0.0f);
    presets->setSpacing(kSpacing);
    mDraft = presets->createChild<rt::Button>(ct::String("Draft"));
    mDraft->setStretch(1.0f);
    mDraft->setTooltip("1024 map, 2048 shadow map, 4 samples over a 2 degree sun. Sets the "
                       "LightmapGI quality to Low, the maximum texture size to 2048 and turns "
                       "the denoiser off.");
    mDraft->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->draftPreset();
            });
    mFinal = presets->createChild<rt::Button>(ct::String("Final"));
    mFinal->setStretch(1.0f);
    mFinal->setTooltip("4096 map, 8192 shadow map, 16 samples over a 2 degree sun. Sets the "
                       "LightmapGI quality to High, the maximum texture size to 4096 and turns "
                       "the denoiser off.");
    mFinal->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->finalPreset();
            });

    for (int i = 0; i < kFieldCount; ++i)
    {
        rt::BoxLayout* row = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        row->setPadding(0.0f);
        row->setSpacing(kSpacing);
        rt::Label* label = row->createChild<rt::Label>(ct::String(kFields[i].label));
        label->setSize(kLabelWidth, 0.0f);
        label->setLayoutAlign(rt::Align::Center);
        NumberField* field = row->createChild<NumberField>();
        field->setStretch(1.0f);
        field->setRange(kFields[i].minimum, kFields[i].maximum);
        field->setDragSensitivity(kFields[i].sensitivity);
        field->setTooltip(ct::String(kFields[i].tip));
        const Field which = static_cast<Field>(i);
        field->valueChanged.connect(
                [link, which](float value)
                {
                    if (link->owner && !link->owner->mSyncing)
                        link->owner->fieldChanged(which, value);
                });
        mFields[i] = field;
    }

    syncFields();
    return true;
}

bool LightmapPanel::attached() const
{
    return mRoot != nullptr;
}

void LightmapPanel::setRevealHandler(const RevealHandler& handler)
{
    mReveal = handler;
}

void LightmapPanel::reveal()
{
    if (mReveal) mReveal();
}

LightmapBaker& LightmapPanel::baker() const
{
    return mBaker;
}

void LightmapPanel::fail(const ct::String& error)
{
    mError = error;
}

void LightmapPanel::clearError()
{
    mError.clear();
}

const ct::String& LightmapPanel::error() const
{
    return mError;
}

bool LightmapPanel::createNode()
{
    clearError();
    ct::String error;
    if (mBaker.busy()) return false;
    const bool created = mBaker.ensureLightmapNode(error);
    if (!created) fail(error);
    return created;
}

bool LightmapPanel::setSunStatic()
{
    clearError();
    ct::String error;
    if (mBaker.busy()) return false;
    const bool done = mBaker.setSunStatic(error);
    if (!done) fail(error);
    return done;
}

bool LightmapPanel::markSelectedStatic()
{
    clearError();
    ct::String error;
    if (mBaker.busy()) return false;
    const bool done = mBaker.markStatic(mSelection.selectedNodes(), error);
    if (!done) fail(error);
    return done;
}

bool LightmapPanel::unwrapMissing()
{
    clearError();
    ct::Vector<MeshInstance3D*> missing;
    const ct::Vector<MeshInstance3D*> meshes = mBaker.meshes();
    for (std::size_t i = 0; i < meshes.size(); ++i)
        if (LightmapMeshIO::isStaticTriangleMesh(meshes[i]->getMesh()) &&
                !LightmapMeshIO::hasUv2(meshes[i]->getMesh()))
            missing.push_back(meshes[i]);
    ct::String error;
    if (missing.empty())
    {
        fail("Every Static mesh already has a second UV set.");
        return false;
    }
    const bool started = mBaker.startUnwrap(missing, error);
    if (!started) fail(error);
    return started;
}

bool LightmapPanel::unwrapRow(int row)
{
    clearError();
    if (row < 0 || row >= static_cast<int>(mRowList.size())) return false;
    Node* scene = mDocument.scene();
    Node* node = scene ? scene->getNode(mRowList[row].path) : nullptr;
    MeshInstance3D* mesh = node ? node->castTo<MeshInstance3D>() : nullptr;
    if (!mesh) return false;
    ct::Vector<MeshInstance3D*> list;
    list.push_back(mesh);
    ct::String error;
    const bool started = mBaker.startUnwrap(list, error);
    if (!started) fail(error);
    return started;
}

bool LightmapPanel::bake()
{
    clearError();
    ct::String error;
    const bool started = mBaker.startBake(error);
    if (!started) fail(error);
    return started;
}

void LightmapPanel::cancel()
{
    mBaker.cancel();
}

void LightmapPanel::draftPreset()
{
    if (mBaker.busy()) return;
    mBaker.applyPreset(true);
    syncFields();
}

void LightmapPanel::finalPreset()
{
    if (mBaker.busy()) return;
    mBaker.applyPreset(false);
    syncFields();
}

void LightmapPanel::fieldChanged(Field which, float value)
{
    LightmapBaker::Settings& settings = mBaker.settings();
    switch (which)
    {
        case kResolution:
            settings.resolution = static_cast<std::uint32_t>(value + 0.5f);
            break;
        case kPadding:
            settings.padding = static_cast<std::uint32_t>(value + 0.5f);
            break;
        case kShadowResolution:
            settings.bake.shadowResolution = static_cast<std::uint32_t>(value + 0.5f);
            break;
        case kSamples:
            settings.bake.sampleCount = static_cast<std::uint32_t>(value + 0.5f);
            break;
        case kSunRadius:
            settings.bake.sunAngularRadius = value;
            break;
        case kFilterRadius:
            settings.bake.filterRadius = value;
            break;
        case kBiasTexels:
            settings.bake.biasTexels = value;
            break;
        case kAmbientGround:
            settings.bake.ambientGround = value;
            break;
        default:
            break;
    }
}

void LightmapPanel::syncFields()
{
    const LightmapBaker::Settings& settings = mBaker.settings();
    const float values[kFieldCount] = { static_cast<float>(settings.resolution),
        static_cast<float>(settings.padding), static_cast<float>(settings.bake.shadowResolution),
        static_cast<float>(settings.bake.sampleCount), settings.bake.sunAngularRadius,
        settings.bake.filterRadius, settings.bake.biasTexels, settings.bake.ambientGround };
    mSyncing = true;
    for (int i = 0; i < kFieldCount; ++i)
        if (mFields[i] && !mFields[i]->isActive()) mFields[i]->setValue(values[i]);
    mSyncing = false;
}

std::uint64_t LightmapPanel::rowSignature(const ct::Vector<MeshInstance3D*>& meshes) const
{
    std::uint64_t hash = 1469598103934665603ull;
    hash = mix(hash, meshes.size());
    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
        hash = hashString(hash, meshes[i]->getName());
        hash = mix(hash, meshes[i]->getMesh().bits());
        hash = mix(hash, LightmapMeshIO::hasUv2(meshes[i]->getMesh()) ? 1 : 0);
        hash = mix(hash, LightmapMeshIO::isStaticTriangleMesh(meshes[i]->getMesh()) ? 1 : 0);
    }
    return hash;
}

void LightmapPanel::rebuildRows(const ct::Vector<MeshInstance3D*>& meshes)
{
    const ct::Vector<rt::Widget*> children = mRows->children();
    for (std::size_t i = 0; i < children.size(); ++i) mRows->removeChild(children[i]);
    mRowList.clear();
    const ct::Rc<Link> link = mLink;
    const rt::Theme& theme = rt::Theme::instance();
    Node* scene = mDocument.scene();
    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
        Row row;
        row.path = scene ? scene->getPathTo(meshes[i]) : ct::String();
        row.name = meshes[i]->getName();
        row.uv2 = LightmapMeshIO::hasUv2(meshes[i]->getMesh());
        row.bakeable = LightmapMeshIO::isStaticTriangleMesh(meshes[i]->getMesh());
        rt::BoxLayout* box = mRows->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        box->setPadding(0.0f);
        box->setSpacing(kSpacing);
        row.label = box->createChild<rt::Label>(row.name);
        row.label->setStretch(1.0f);
        row.label->setLayoutAlign(rt::Align::Center);
        row.state = box->createChild<rt::Label>(
                ct::String(!row.bakeable ? "not bakeable" : (row.uv2 ? "UV2" : "no UV2")));
        row.state->setColor(row.uv2 && row.bakeable ? theme.textDisabled : theme.textColor);
        row.state->setLayoutAlign(rt::Align::Center);
        row.button = box->createChild<rt::Button>(ct::String("Unwrap"));
        row.button->setTooltip("Generate the second UV set of this mesh");
        const int index = static_cast<int>(i);
        row.button->clicked.connect(
                [link, index]
                {
                    if (link->owner) link->owner->unwrapRow(index);
                });
        mRowList.push_back(row);
    }
    requestLayout();
}

void LightmapPanel::update()
{
    if (!mRoot) return;
    mBaker.update();

    LightmapGI* gi = mBaker.lightmapNode();
    DirectionalLight3D* sun = mBaker.sun();
    const bool busy = mBaker.busy();
    const bool editing = !mDocument.playing() && mDocument.scene() != nullptr;

    const ct::String nodeText =
            gi ? ct::String("LightmapGI: ") + gi->getName() : ct::String(kNoNodeText);
    if (mNodeLabel->text() != nodeText) mNodeLabel->setText(nodeText);
    ct::String dataText;
    if (gi)
        dataText = ct::String("data: ") +
                   (gi->getLightDataPath().empty() ? ct::String("none") : gi->getLightDataPath());
    if (mDataLabel->text() != dataText) mDataLabel->setText(dataText);
    const ct::String nodeButtonText = gi ? "Select LightmapGI" : "Add LightmapGI";
    if (mNodeButton->text() != nodeButtonText)
    {
        mNodeButton->setText(nodeButtonText);
        mNodeButton->setIconId(gi ? rt::IconId::None : EditorIcons::get("Add"));
    }
    mNodeButton->setEnabled(editing && !busy);

    ct::String sunText = kNoSunText;
    if (sun)
    {
        sunText = ct::String("Sun: ") + sun->getName() +
                  (sun->getBakeMode() == LightBakeMode::Static ? ", bake mode Static"
                                                               : ", bake mode not Static");
    }
    if (mSunLabel->text() != sunText) mSunLabel->setText(sunText);
    mSunButton->setEnabled(editing && !busy && sun && !mBaker.sunIsStatic());

    const ct::String ambientText = ct::String("Ambient: ") + colorText(mBaker.ambient());
    if (mAmbientLabel->text() != ambientText) mAmbientLabel->setText(ambientText);

    const ct::Vector<MeshInstance3D*> meshes = mBaker.meshes();
    const std::uint64_t signature = rowSignature(meshes);
    if (signature != mSignature)
    {
        mSignature = signature;
        rebuildRows(meshes);
    }
    ct::String meshesText;
    meshesText.append_number(static_cast<unsigned long long>(meshes.size()));
    meshesText += meshes.size() == 1 ? " Static mesh" : " Static meshes";
    if (mMeshesLabel->text() != meshesText) mMeshesLabel->setText(meshesText);
    for (std::size_t i = 0; i < mRowList.size(); ++i)
        mRowList[i].button->setEnabled(editing && !busy && mRowList[i].bakeable);
    mStaticButton->setEnabled(editing && !busy);

    mDraft->setEnabled(editing && !busy && gi);
    mFinal->setEnabled(editing && !busy && gi);
    for (int i = 0; i < kFieldCount; ++i) mFields[i]->setEnabled(!busy);
    syncFields();
    mUnwrap->setEnabled(editing && !busy && !meshes.empty());
    mBake->setEnabled(editing && !busy && sun && !meshes.empty());
    mCancel->setEnabled(busy);

    mProgress->setValue(busy ? mBaker.progress() : 0.0f);
    ct::String status = busy ? mBaker.message() : ct::String();
    if (!mError.empty()) status = mError;
    else if (!busy && !mBaker.summary().text.empty())
        status = mBaker.summary().text;
    if (mStatus->text() != status) mStatus->setText(status);
    ct::String summary;
    if (!busy && mBaker.summary().seconds > 0.0)
    {
        char text[96];
        snprintf(text, sizeof(text), "%.2f s%s", mBaker.summary().seconds,
                mBaker.summary().ok ? "" : " (did not finish)");
        summary = text;
    }
    if (mSummary->text() != summary) mSummary->setText(summary);
}

int LightmapPanel::rowCount() const
{
    return static_cast<int>(mRowList.size());
}

ct::String LightmapPanel::rowName(int row) const
{
    return row >= 0 && row < rowCount() ? mRowList[row].name : ct::String();
}

bool LightmapPanel::rowHasUv2(int row) const
{
    return row >= 0 && row < rowCount() && mRowList[row].uv2;
}

rt::Button* LightmapPanel::rowButton(int row) const
{
    return row >= 0 && row < rowCount() ? mRowList[row].button : nullptr;
}

rt::Button* LightmapPanel::nodeButton() const
{
    return mNodeButton;
}

rt::Button* LightmapPanel::sunButton() const
{
    return mSunButton;
}

rt::Button* LightmapPanel::staticButton() const
{
    return mStaticButton;
}

rt::Button* LightmapPanel::draftButton() const
{
    return mDraft;
}

rt::Button* LightmapPanel::finalButton() const
{
    return mFinal;
}

rt::Button* LightmapPanel::unwrapButton() const
{
    return mUnwrap;
}

rt::Button* LightmapPanel::bakeButton() const
{
    return mBake;
}

rt::Button* LightmapPanel::cancelButton() const
{
    return mCancel;
}

rt::Label* LightmapPanel::nodeLabel() const
{
    return mNodeLabel;
}

rt::Label* LightmapPanel::dataLabel() const
{
    return mDataLabel;
}

rt::Label* LightmapPanel::sunLabel() const
{
    return mSunLabel;
}

rt::Label* LightmapPanel::ambientLabel() const
{
    return mAmbientLabel;
}

rt::Label* LightmapPanel::statusLabel() const
{
    return mStatus;
}

rt::Label* LightmapPanel::summaryLabel() const
{
    return mSummary;
}

rt::ProgressBar* LightmapPanel::progressBar() const
{
    return mProgress;
}

NumberField* LightmapPanel::field(Field which) const
{
    return which >= 0 && which < kFieldCount ? mFields[which] : nullptr;
}

rt::ScrollView* LightmapPanel::scrollView() const
{
    return mScroll;
}

} // namespace prisma3d
