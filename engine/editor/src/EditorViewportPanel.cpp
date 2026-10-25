#include "prisma3d/editor/EditorViewportPanel.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

#include "platform.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/ViewWidgets.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const Math::Vec4 kClearColor(0.16f, 0.17f, 0.19f, 1.0f);
const float kLabelInset = 8.0f;

const char* const kButtonTooltips[EditorViewportPanel::kButtonCount] = { "Select Mode (V)",
    "Transform Mode (Q)", "Move Mode (W)", "Rotate Mode (E)", "Scale Mode (R)",
    "Terrain Brushes (B)", "Use Local Space (T)", "Use Snap (Y)", "Configure Snap...", "Show Grid",
    "Show Gizmos (lights, cameras, shapes, icons)", "Orthogonal View (Numpad 5)" };

const char* const kButtonIcons[EditorViewportPanel::kButtonCount] = { "ToolSelect", "Marker3D",
    "ToolMove", "ToolRotate", "ToolScale", "HeightMapShape3D", "Object", "Snap", "Tools", "Grid",
    "GizmoLight", "Camera3D" };

unsigned int buttonBit(MouseButton button)
{
    return mouseButtonToMask(button);
}

} // namespace

class ViewportImage : public rt::Widget
{
public:
    IG_WIDGET_CLASS(ViewportImage, rt::Widget);

    void setTexture(rt::TextureHandle texture)
    {
        if (mTexture.value == texture.value) return;
        mTexture = texture;
        markDirty();
    }

    rt::TextureHandle texture() const
    {
        return mTexture;
    }

    using DropHandler = ct::Function<void(const ct::String&, float, float)>;

    void setDropHandler(const DropHandler& handler)
    {
        mDrop = handler;
    }

    bool acceptsDrop(const rt::DragPayload& payload) override
    {
        ct::String path;
        if (!mDrop || !AssetGrid::payloadPath(payload, path)) return false;
        const AssetKind kind = AssetTypes::classify(AssetTypes::filePart(path), false);
        return kind == AssetKind::Model || kind == AssetKind::Prefab ||
               kind == AssetKind::Material || kind == AssetKind::Script;
    }

    void onDropReceive(const rt::DragPayload& payload) override
    {
        ct::String path;
        if (mDrop && AssetGrid::payloadPath(payload, path))
            mDrop(path, payload.dropX, payload.dropY);
    }

    void setRotationControl(const ViewportRotationControl* control)
    {
        mRotation = control;
    }

    void setLabel(const ct::String& label)
    {
        if (mLabel == label) return;
        mLabel = label;
        markDirty();
    }

    const ct::String& label() const
    {
        return mLabel;
    }

    Vec2f sizeHint() const override
    {
        return { 64.0f, 64.0f };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;

        ctx.pushClip(abs);
        if (mTexture) ctx.drawImage(mTexture, abs, rt::Rect{ 0.0f, 1.0f, 1.0f, -1.0f });

        const rt::Theme& theme = rt::Theme::instance();
        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.font.SetFontSize(theme.fontSize);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        ctx.font.SetColor(rt::Color(240, 240, 240, 255));
        ctx.font.Print(mLabel.c_str(), abs.x + kLabelInset, abs.y + kLabelInset + ascender);
        if (mRotation) mRotation->draw(ctx, abs.x, abs.y);
        if (overrideFont_) ctx.popFont();
        ctx.popClip();
    }

private:
    rt::TextureHandle mTexture;
    ct::String mLabel;
    DropHandler mDrop;
    const ViewportRotationControl* mRotation = nullptr;
};

class SnapDialog : public rt::FloatWindow
{
public:
    IG_WIDGET_CLASS(SnapDialog, rt::FloatWindow);

    explicit SnapDialog(EditorGizmo& gizmo)
            : rt::FloatWindow("Configure Snap"),
              mGizmo(gizmo)
    {
        setResizable(false);
        setMinimizable(false);

        rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
        main->setSpacing(6.0f);
        main->setPadding(12.0f);

        main->createChild<rt::Label>("Translate Snap (m)");
        mTranslate = main->createChild<NumberField>();
        mTranslate->setDragSensitivity(0.01f);
        mTranslate->setValue(gizmo.settings().translateSnap);
        main->createChild<rt::Label>("Rotate Snap (degrees)");
        mRotate = main->createChild<NumberField>();
        mRotate->setDragSensitivity(0.1f);
        mRotate->setValue(gizmo.settings().rotateSnap);
        main->createChild<rt::Label>("Scale Snap (%)");
        mScale = main->createChild<NumberField>();
        mScale->setDragSensitivity(1.0f);
        mScale->setValue(gizmo.settings().scaleSnap);

        mTranslate->valueChanged.connect([this](float value)
                { mGizmo.settings().translateSnap = value < 0.0f ? 0.0f : value; });
        mRotate->valueChanged.connect([this](float value)
                { mGizmo.settings().rotateSnap = value < 0.0f ? 0.0f : value; });
        mScale->valueChanged.connect(
                [this](float value) { mGizmo.settings().scaleSnap = value < 0.0f ? 0.0f : value; });

        const float width = 240.0f;
        const float height = 240.0f;
        setFloatSize(width, height);
        const rt::IO& io = rt::GetIO();
        setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);
    }

private:
    EditorGizmo& mGizmo;
    NumberField* mTranslate;
    NumberField* mRotate;
    NumberField* mScale;
};

EditorViewportPanel::EditorViewportPanel(RenderingServer& server)
        : mServer(server),
          mLink(ct::make_rc<Link>(Link{ this })),
          mCamera(),
          mCameraId(),
          mViewport(),
          mScenario(),
          mUpdateMode(ViewportUpdateMode::Disabled),
          mView(nullptr),
          mColumn(nullptr),
          mToolBar(nullptr),
          mGizmo(nullptr),
          mRotation(mCamera),
          mButtons{},
          mShowGizmos(true),
          mTexture(),
          mOrthogonalButton(-1),
          mWidth(0),
          mHeight(0),
          mResizes(0),
          mDragButtons(0),
          mCaptured(false),
          mMouseX(0),
          mMouseY(0),
          mSavedMouseX(0),
          mSavedMouseY(0),
          mPreDraw(false),
          mLastTime(0),
          mSyncing(false),
          mSuspended(false),
          mTerrainTool()
{
    mCameraId = mServer.cameraCreate();
    mViewport = mServer.viewportCreate();
    mServer.viewportAttachCamera(mViewport, mCameraId);
    mServer.viewportSetClearColor(mViewport, kClearColor);
    mServer.viewportSetSrgbOutput(mViewport, true);
    mServer.viewportSetUpdateMode(mViewport, ViewportUpdateMode::Disabled);
    mServer.viewportSetDrawBatch(mViewport, true);
    for (int& id: mButtons) id = -1;
    applyCamera();
}

EditorViewportPanel::~EditorViewportPanel()
{
    mLink->owner = nullptr;
    disconnectPreDraw();
    delete mGizmo;
    mGizmo = nullptr;
    if (mWidth > 0) Main::setOverlaySize(0, 0);
    if (mCaptured)
    {
        if (Device* device = Device::singleton()) device->mouseSetMode(MouseMode::Visible);
        mCaptured = false;
    }
    mServer.viewportFree(mViewport);
    mServer.cameraFree(mCameraId);
}

bool EditorViewportPanel::attach(rt::Widget* parent)
{
    if (mView || !parent) return false;
    mColumn = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setPadding(0.0f);
    mColumn->setSpacing(0.0f);
    mToolBar = mColumn->createChild<rt::ToolBar>();
    mToolBar->setDraggable(false);
    mTerrainTool.attach(mColumn);
    mView = mColumn->createChild<ViewportImage>();
    mView->setStretch(1.0f);
    mView->setRotationControl(&mRotation);
    mToolBar->setButtonSize(28.0f);
    mToolBar->setIconPadding(6.0f);

    for (int i = 0; i < kButtonCount; ++i)
    {
        if (i == kLocalSpace || i == kGrid) mToolBar->addSeparator();
        const bool toggle = i != kSnapSettings;
        const rt::IconId icon = buttonIcon(static_cast<Button>(i));
        mButtons[i] = toggle ? mToolBar->addToggle(icon, kButtonTooltips[i], false)
                             : mToolBar->addButton(icon, kButtonTooltips[i]);
        mToolBar->setItemDraggable(mButtons[i], false);
    }
    mOrthogonalButton = mButtons[kOrthogonal];
    mToolBar->setItemChecked(mButtons[kGrid], mGrid.gridEnabled());
    mToolBar->setItemChecked(mButtons[kGizmos], mShowGizmos);
    mToolBar->setItemChecked(mButtons[kTransform], true);

    const ct::Rc<Link> link = mLink;
    mToolBar->toggleChanged.connect(
            [link](int id, bool checked)
            {
                if (link->owner) link->owner->toggledButton(id, checked);
            });
    mToolBar->buttonClicked.connect(
            [link](int id)
            {
                if (link->owner) link->owner->clickedButton(id);
            });
    mView->setLabel(mCamera.viewTypeName());
    mView->setDropHandler(
            [link](const ct::String& path, float x, float y)
            {
                if (link->owner) link->owner->assetDropped(path, x, y);
            });
    return true;
}

bool EditorViewportPanel::attached() const
{
    return mView != nullptr;
}

void EditorViewportPanel::setCameraAspectProvider(const ct::Function<float()>& provider)
{
    mAspectProvider = provider;
}

EditorCameraGizmo& EditorViewportPanel::cameraGizmo()
{
    return mCameraGizmo;
}

EditorLightGizmo& EditorViewportPanel::lightGizmo()
{
    return mLightGizmo;
}

EditorShapeGizmo& EditorViewportPanel::shapeGizmo()
{
    return mShapeGizmo;
}

EditorShapeHandles& EditorViewportPanel::shapeHandles()
{
    return mShapeHandles;
}

EditorNodeIcons& EditorViewportPanel::nodeIcons()
{
    return mNodeIcons;
}

TerrainEditTool& EditorViewportPanel::terrainTool()
{
    return mTerrainTool;
}

void EditorViewportPanel::setScenario(ScenarioId scenario)
{
    mScenario = scenario;
    mServer.viewportSetScenario(mViewport, scenario);
}

ScenarioId EditorViewportPanel::scenario() const
{
    return mScenario;
}

void EditorViewportPanel::setClearColor(const Math::Vec4& color)
{
    mServer.viewportSetClearColor(mViewport, color);
}

void EditorViewportPanel::connectPreDraw()
{
    if (mPreDraw) return;
    mPreDraw = true;
    mLastTime = time_nanos();
    const ct::Rc<Link> link = mLink;
    mServer.framePreDrawConnect(this,
            [link]()
            {
                EditorViewportPanel* owner = link->owner;
                if (!owner) return;
                const std::uint64_t now = time_nanos();
                const float delta = static_cast<float>(now - owner->mLastTime) * 1e-9f;
                owner->mLastTime = now;
                owner->update(delta);
            });
}

void EditorViewportPanel::disconnectPreDraw()
{
    if (!mPreDraw) return;
    mPreDraw = false;
    mServer.framePreDrawDisconnect(this);
}

bool EditorViewportPanel::visibleInTree() const
{
    for (const rt::Widget* widget = mView; widget; widget = widget->parent())
        if (!widget->isVisible()) return false;
    return true;
}

void EditorViewportPanel::syncSize()
{
    const rt::Rect rect = mView->absoluteRect();
    const std::uint32_t width = static_cast<std::uint32_t>(rect.w + 0.5f);
    const std::uint32_t height = static_cast<std::uint32_t>(rect.h + 0.5f);
    if (width == 0 || height == 0 || (width == mWidth && height == mHeight)) return;
    mWidth = width;
    mHeight = height;
    mServer.viewportSetSize(mViewport, width, height);
    ++mResizes;
}

void EditorViewportPanel::applyCamera()
{
    const EditorCamera::Projection projection = mCamera.projection();
    if (projection.orthogonal)
        mServer.cameraSetOrthogonal(mCameraId, projection.size, projection.zNear, projection.zFar);
    else
        mServer.cameraSetPerspective(mCameraId, projection.fov, projection.zNear, projection.zFar);
    mServer.cameraSetTransform(mCameraId, mCamera.interpToCameraTransform());
}

void EditorViewportPanel::syncToolBar()
{
    if (!mToolBar || mButtons[kSelect] < 0) return;
    mSyncing = true;
    const auto check = [this](int button, bool checked)
    {
        if (mToolBar->isItemChecked(mButtons[button]) != checked)
            mToolBar->setItemChecked(mButtons[button], checked);
    };
    const EditorGizmo::Tool tool = mGizmo ? mGizmo->tool() : EditorGizmo::Tool::Transform;
    check(kSelect, tool == EditorGizmo::Tool::Select);
    check(kTransform, tool == EditorGizmo::Tool::Transform);
    check(kMove, tool == EditorGizmo::Tool::Move);
    check(kRotate, tool == EditorGizmo::Tool::Rotate);
    check(kScale, tool == EditorGizmo::Tool::Scale);
    check(kTerrain, mTerrainTool.active());
    const bool local = mGizmo && mGizmo->localSpace();
    check(kLocalSpace, local);
    check(kSnap, mGizmo && mGizmo->snapEnabled());
    check(kGrid, mGrid.gridEnabled());
    check(kGizmos, mShowGizmos);
    check(kOrthogonal, mCamera.isOrthogonal());
    mSyncing = false;
}

void EditorViewportPanel::toggledButton(int id, bool checked)
{
    if (mSyncing) return;
    for (int i = 0; i < kButtonCount; ++i)
    {
        if (mButtons[i] != id) continue;
        switch (static_cast<Button>(i))
        {
            case kSelect:
                if (mGizmo) mGizmo->setTool(EditorGizmo::Tool::Select);
                break;
            case kTransform:
                if (mGizmo) mGizmo->setTool(EditorGizmo::Tool::Transform);
                break;
            case kMove:
                if (mGizmo) mGizmo->setTool(EditorGizmo::Tool::Move);
                break;
            case kRotate:
                if (mGizmo) mGizmo->setTool(EditorGizmo::Tool::Rotate);
                break;
            case kScale:
                if (mGizmo) mGizmo->setTool(EditorGizmo::Tool::Scale);
                break;
            case kTerrain:
                mTerrainTool.setActive(checked);
                break;
            case kLocalSpace:
                if (mGizmo) mGizmo->setLocalSpace(checked);
                break;
            case kSnap:
                if (mGizmo) mGizmo->setSnapEnabled(checked);
                break;
            case kGrid:
                mGrid.setGridEnabled(checked);
                break;
            case kGizmos:
                mShowGizmos = checked;
                break;
            case kOrthogonal:
                mCamera.setOrthogonal(checked);
                mCamera.updateCamera();
                break;
            default:
                break;
        }
        syncToolBar();
        return;
    }
}

void EditorViewportPanel::clickedButton(int id)
{
    if (id == mButtons[kSnapSettings]) openSnapSettings();
}

void EditorViewportPanel::openSnapSettings()
{
    if (!mGizmo) return;
    if (snapDialog())
    {
        rt::WidgetApp::instance().bringToFront(snapDialog());
        return;
    }
    rt::WidgetApp::instance().addFloat<SnapDialog>(*mGizmo);
}

rt::FloatWindow* EditorViewportPanel::snapDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (SnapDialog* found = rt::widget_cast<SnapDialog>(floats[i - 1])) return found;
    return nullptr;
}

void EditorViewportPanel::setOrthogonal(bool orthogonal)
{
    mCamera.setOrthogonal(orthogonal);
    mCamera.updateCamera();
    syncToolBar();
}

void EditorViewportPanel::setGridVisible(bool visible)
{
    mGrid.setGridEnabled(visible);
    syncToolBar();
}

void EditorViewportPanel::bindEditor(SceneTree& tree, EditorSelection& selection)
{
    delete mGizmo;
    mGizmo = new EditorGizmo(tree, selection);
    mTerrainTool.setSelection(&selection);
    mShapeHandles.setSelection(&selection);
}

void EditorViewportPanel::setChangeListener(EditorChangeListener* listener)
{
    mTerrainTool.setChangeListener(listener);
    mShapeHandles.setChangeListener(listener);
    if (mGizmo) mGizmo->setChangeListener(listener);
}

EditorGizmo* EditorViewportPanel::gizmo() const
{
    return mGizmo;
}

EditorGrid& EditorViewportPanel::grid()
{
    return mGrid;
}

ViewportRotationControl& EditorViewportPanel::rotationControl()
{
    return mRotation;
}

const ViewportRotationControl& EditorViewportPanel::rotationControl() const
{
    return mRotation;
}

const EditorGrid& EditorViewportPanel::grid() const
{
    return mGrid;
}

rt::ToolBar* EditorViewportPanel::toolBar() const
{
    return mToolBar;
}

rt::Widget* EditorViewportPanel::column() const
{
    return mColumn;
}

int EditorViewportPanel::buttonId(Button button) const
{
    return button >= 0 && button < kButtonCount ? mButtons[button] : -1;
}

rt::IconId EditorViewportPanel::buttonIcon(Button button)
{
    return button >= 0 && button < kButtonCount ? EditorIcons::get(kButtonIcons[button])
                                                : rt::IconId::None;
}

const char* EditorViewportPanel::buttonTooltip(Button button)
{
    return button >= 0 && button < kButtonCount ? kButtonTooltips[button] : "";
}

EditorView EditorViewportPanel::view3d() const
{
    EditorView view;
    view.camera = mCamera.interpToCameraTransform();
    view.projection = mCamera.projection();
    view.width = mWidth > 0 ? static_cast<float>(mWidth) : 1.0f;
    view.height = mHeight > 0 ? static_cast<float>(mHeight) : 1.0f;
    return view;
}

void EditorViewportPanel::setAssetDropHandler(const AssetDropHandler& handler)
{
    mAssetDrop = handler;
}

bool EditorViewportPanel::groundPoint(float x, float y, Math::Vec3& out) const
{
    const EditorView view = view3d();
    const Math::Vec3 origin = view.rayOrigin(x, y);
    const Math::Vec3 direction = view.rayDirection(x, y);
    if (fabsf(direction.y) < 1.0e-6f) return false;
    const float distance = -origin.y / direction.y;
    if (distance <= 0.0f || distance > view.projection.zFar) return false;
    out = origin + direction * distance;
    return true;
}

Math::Vec3 EditorViewportPanel::focusPoint() const
{
    const EditorCamera::Cursor& cursor = mCamera.cursor();
    return Math::Vec3(static_cast<float>(cursor.posX), static_cast<float>(cursor.posY),
            static_cast<float>(cursor.posZ));
}

Math::Vec3 EditorViewportPanel::dropPosition(float screenX, float screenY) const
{
    Math::Vec3 point;
    if (mView)
    {
        const rt::Rect rect = mView->absoluteRect();
        if (groundPoint(screenX - rect.x, screenY - rect.y, point)) return point;
    }
    return focusPoint();
}

void EditorViewportPanel::setMaterialDropHandler(const MaterialDropHandler& handler)
{
    mMaterialDrop = handler;
}

void EditorViewportPanel::setScriptDropHandler(const MaterialDropHandler& handler)
{
    mScriptDrop = handler;
}

Node3D* EditorViewportPanel::pickAt(float screenX, float screenY) const
{
    if (!mView || !mGizmo) return nullptr;
    const rt::Rect rect = mView->absoluteRect();
    if (!rect.contains(screenX, screenY)) return nullptr;
    return mGizmo->pick(screenX - rect.x, screenY - rect.y);
}

void EditorViewportPanel::assetDropped(const ct::String& path, float x, float y)
{
    if (AssetTypes::classify(AssetTypes::filePart(path), false) == AssetKind::Material)
    {
        if (mMaterialDrop) mMaterialDrop(path, pickAt(x, y));
        return;
    }
    if (AssetTypes::classify(AssetTypes::filePart(path), false) == AssetKind::Script)
    {
        if (mScriptDrop) mScriptDrop(path, pickAt(x, y));
        return;
    }
    if (mAssetDrop) mAssetDrop(path, dropPosition(x, y));
}

void EditorViewportPanel::drawOverlays()
{
    BatchRenderer& batch = mServer.batch();
    const EditorView view = view3d();
    mGrid.draw(batch, view);
    if (mGizmo && mShowGizmos)
    {
        mCameraGizmo.draw(batch, mGizmo->sceneRoot(), &mGizmo->selection(),
                mAspectProvider ? mAspectProvider() : 1.0f);
        mLightGizmo.draw(batch, mGizmo->sceneRoot(), &mGizmo->selection(), view);
        mShapeGizmo.draw(batch, mGizmo->sceneRoot(), &mGizmo->selection(), view);
        mNodeIcons.settings() = mGizmo->iconSettings();
        mNodeIcons.draw(batch, mGizmo->sceneRoot(), &mGizmo->selection(), view);
    }
    if (mGizmo)
    {
        mShapeHandles.draw(batch);
        mTerrainTool.draw(batch);
        mGizmo->setView(view);
        mGizmo->update();
        mGizmo->draw(batch);
    }
}

void EditorViewportPanel::syncMouseCapture()
{
    Device* device = Device::singleton();
    const bool freelook = mCamera.freelookEnabled();
    if (freelook && !mCaptured)
    {
        mCaptured = true;
        mSavedMouseX = mMouseX;
        mSavedMouseY = mMouseY;
        if (device) device->mouseSetMode(MouseMode::Captured);
    }
    else if (!freelook && mCaptured)
    {
        mCaptured = false;
        if (device)
        {
            device->mouseSetMode(MouseMode::Visible);
            device->warpMouse(mSavedMouseX, mSavedMouseY);
        }
    }
}

void EditorViewportPanel::setSuspended(bool suspended)
{
    mSuspended = suspended;
}

bool EditorViewportPanel::suspended() const
{
    return mSuspended;
}

void EditorViewportPanel::update(float delta)
{
    if (!mView || !mViewport.valid()) return;

    const rt::Rect rect = mView->absoluteRect();
    const bool visible = !mSuspended && visibleInTree() && rect.w >= 1.0f && rect.h >= 1.0f;
    if (!visible)
    {
        if (mUpdateMode != ViewportUpdateMode::Disabled)
        {
            mUpdateMode = ViewportUpdateMode::Disabled;
            mServer.viewportSetUpdateMode(mViewport, mUpdateMode);
        }
        return;
    }

    syncSize();
    if (mWidth == 0 || mHeight == 0) return;
    mRotation.setViewSize(static_cast<float>(mWidth), static_cast<float>(mHeight));
    if (mUpdateMode != ViewportUpdateMode::Always)
    {
        mUpdateMode = ViewportUpdateMode::Always;
        mServer.viewportSetUpdateMode(mViewport, mUpdateMode);
    }

    mCamera.update(delta);
    applyCamera();
    syncToolBar();
    syncMouseCapture();
    if (mGizmo) mTerrainTool.setSceneRoot(mGizmo->sceneRoot());
    mTerrainTool.update(delta, view3d());
    mShapeHandles.update(view3d());
    Main::setOverlaySize(static_cast<int>(mWidth), static_cast<int>(mHeight));
    drawOverlays();

    mTexture = mServer.viewportGetTexture(mViewport);
    mView->setTexture(WidgetHost::textureHandle(mTexture));
    mView->setLabel(mCamera.viewTypeName());
    mView->markDirty();
}

bool EditorViewportPanel::hovered(int x, int y) const
{
    if (!mView || !visibleInTree()) return false;
    const float px = static_cast<float>(x);
    const float py = static_cast<float>(y);
    if (!mView->absoluteRect().contains(px, py)) return false;
    if (const rt::WidgetApp* app = rt::WidgetApp::existing())
    {
        if (app->popup() && app->popup()->popupContains(px, py)) return false;
        const ct::Vector<rt::FloatWindow*>& floats = app->floats();
        for (std::size_t i = 0; i < floats.size(); ++i)
            if (floats[i]->isVisible() && floats[i]->absoluteRect().contains(px, py)) return false;
    }
    return true;
}

bool EditorViewportPanel::typing() const
{
    const rt::WidgetApp* app = rt::WidgetApp::existing();
    return (app != nullptr && app->focused() != nullptr) || WidgetHost::modalOpen();
}

bool EditorViewportPanel::wants(const InputEvent& event) const
{
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            return mDragButtons != 0 || mCamera.freelookEnabled() ||
                   (event.mouseMotion.buttonMask == 0 &&
                           hovered(event.mouseMotion.x, event.mouseMotion.y));
        case InputEventType::MouseButton:
            return !event.mouseButton.pressed || hovered(event.mouseButton.x, event.mouseButton.y);
        case InputEventType::MouseWheel:
            return hovered(event.mouseWheel.x, event.mouseWheel.y);
        case InputEventType::Key:
            if (!event.key.pressed) return true;
            if (typing()) return false;
            return mDragButtons != 0 || mCamera.freelookEnabled() || hovered(mMouseX, mMouseY);
        default:
            return false;
    }
}

bool EditorViewportPanel::gizmoWants(const InputEvent& event) const
{
    switch (event.type)
    {
        case InputEventType::MouseButton:
            if (event.mouseButton.button == MouseButton::Left)
            {
                if (!event.mouseButton.pressed) return true;
                EditorCamera probe = mCamera;
                probe.setModifiers(event.mouseButton.modifiers);
                return !probe.navButtonWithModifiers(EditorCamera::NavigationButton::Left);
            }
            return event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                   mGizmo->editing();
        case InputEventType::MouseMotion:
            return !mCamera.freelookEnabled();
        case InputEventType::Key:
            if (mCamera.freelookEnabled() || mCamera.isNavigating()) return false;
            return true;
        default:
            return false;
    }
}

bool EditorViewportPanel::handleEvent(const InputEvent& event)
{
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            mMouseX = event.mouseMotion.x;
            mMouseY = event.mouseMotion.y;
            break;
        case InputEventType::MouseButton:
            mMouseX = event.mouseButton.x;
            mMouseY = event.mouseButton.y;
            break;
        case InputEventType::MouseWheel:
            mMouseX = event.mouseWheel.x;
            mMouseY = event.mouseWheel.y;
            break;
        default:
            break;
    }
    if (mView && event.type == InputEventType::MouseMotion && !mRotation.orbiting() &&
            !hovered(event.mouseMotion.x, event.mouseMotion.y))
        mRotation.mouseExit();
    if (!mView || !wants(event)) return false;

    if (event.type == InputEventType::MouseButton)
    {
        if (event.mouseButton.pressed)
        {
            mDragButtons |= buttonBit(event.mouseButton.button);
            if (rt::WidgetApp* app = rt::WidgetApp::existing())
                if (app->focused()) app->setFocused(nullptr);
        }
        else
            mDragButtons &= ~buttonBit(event.mouseButton.button);
    }

    {
        const rt::Rect rect = mView->absoluteRect();
        mRotation.setViewSize(rect.w, rect.h);
        if (mRotation.handleEvent(event, static_cast<float>(mMouseX) - rect.x,
                    static_cast<float>(mMouseY) - rect.y))
            return true;
    }

    if (event.type == InputEventType::Key && event.key.pressed && !event.key.echo &&
            event.key.key == Key::B && !mCamera.freelookEnabled() && !mCamera.isNavigating() &&
            (event.key.modifiers & (kModControl | kModAlt)) == 0)
    {
        mTerrainTool.setActive(!mTerrainTool.active());
        syncToolBar();
        return true;
    }

    if (!mTerrainTool.active() && !mCamera.freelookEnabled() &&
            (mShapeHandles.target() || mShapeHandles.dragging()))
    {
        const rt::Rect rect = mView->absoluteRect();
        bool navigating = false;
        if (event.type == InputEventType::MouseButton &&
                event.mouseButton.button == MouseButton::Left && event.mouseButton.pressed)
        {
            EditorCamera probe = mCamera;
            probe.setModifiers(event.mouseButton.modifiers);
            navigating = probe.navButtonWithModifiers(EditorCamera::NavigationButton::Left);
        }
        if (mShapeHandles.handleEvent(event, static_cast<float>(mMouseX) - rect.x,
                    static_cast<float>(mMouseY) - rect.y, navigating))
            return true;
    }

    if (mTerrainTool.active() && !mCamera.freelookEnabled())
    {
        const rt::Rect rect = mView->absoluteRect();
        bool navigating = false;
        if (event.type == InputEventType::MouseButton &&
                event.mouseButton.button == MouseButton::Left && event.mouseButton.pressed)
        {
            EditorCamera probe = mCamera;
            probe.setModifiers(event.mouseButton.modifiers);
            navigating = probe.navButtonWithModifiers(EditorCamera::NavigationButton::Left);
        }
        if (mTerrainTool.handleEvent(event, static_cast<float>(mMouseX) - rect.x,
                    static_cast<float>(mMouseY) - rect.y, navigating))
            return true;
    }

    if (mGizmo && gizmoWants(event))
    {
        const rt::Rect rect = mView->absoluteRect();
        if (mGizmo->handleEvent(event, static_cast<float>(mMouseX) - rect.x,
                    static_cast<float>(mMouseY) - rect.y))
        {
            syncToolBar();
            return true;
        }
    }

    const bool consumed = mCamera.inputEvent(event);
    syncMouseCapture();
    return consumed;
}

EditorCamera& EditorViewportPanel::camera()
{
    return mCamera;
}

const EditorCamera& EditorViewportPanel::camera() const
{
    return mCamera;
}

ViewportId EditorViewportPanel::viewportId() const
{
    return mViewport;
}

CameraId EditorViewportPanel::cameraId() const
{
    return mCameraId;
}

TextureId EditorViewportPanel::texture() const
{
    return mTexture;
}

rt::TextureHandle EditorViewportPanel::shownTexture() const
{
    return mView ? mView->texture() : rt::TextureHandle();
}

rt::Widget* EditorViewportPanel::view() const
{
    return mView;
}

int EditorViewportPanel::orthogonalButton() const
{
    return mOrthogonalButton;
}

std::uint32_t EditorViewportPanel::width() const
{
    return mWidth;
}

std::uint32_t EditorViewportPanel::height() const
{
    return mHeight;
}

unsigned int EditorViewportPanel::resizeCount() const
{
    return mResizes;
}

bool EditorViewportPanel::rendering() const
{
    return mUpdateMode == ViewportUpdateMode::Always;
}

bool EditorViewportPanel::dragging() const
{
    return mDragButtons != 0;
}

} // namespace prisma3d
