#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorCamera.h"
#include "prisma3d/editor/EditorCameraGizmo.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorGrid.h"
#include "prisma3d/editor/EditorLightGizmo.h"
#include "prisma3d/editor/EditorNodeIcons.h"
#include "prisma3d/editor/EditorShapeGizmo.h"
#include "prisma3d/editor/EditorShapeHandles.h"
#include "prisma3d/editor/TerrainEditTool.h"
#include "prisma3d/editor/ViewportRotationControl.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/ToolBar.hpp>
#include <igui/widgets/ViewWidgets.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorSelection;
class Node;
class Node3D;
class SceneTree;
class ViewportImage;

class EditorViewportPanel
{
public:
    explicit EditorViewportPanel(RenderingServer& server);
    ~EditorViewportPanel();
    EditorViewportPanel(const EditorViewportPanel&) = delete;
    EditorViewportPanel& operator=(const EditorViewportPanel&) = delete;

    enum Button : int
    {
        kSelect,
        kTransform,
        kMove,
        kRotate,
        kScale,
        kTerrain,
        kLocalSpace,
        kSnap,
        kSnapSettings,
        kGrid,
        kGizmos,
        kOrthogonal,
        kButtonCount
    };

    bool attach(ig::retained::Widget* parent);
    bool attached() const;

    void bindEditor(SceneTree& tree, EditorSelection& selection);
    void setChangeListener(EditorChangeListener* listener);
    EditorGizmo* gizmo() const;
    EditorGrid& grid();
    ViewportRotationControl& rotationControl();
    const ViewportRotationControl& rotationControl() const;
    const EditorGrid& grid() const;
    EditorView view3d() const;
    ig::retained::ToolBar* toolBar() const;
    int buttonId(Button button) const;
    static const char* buttonTooltip(Button button);
    static ig::retained::IconId buttonIcon(Button button);
    void openSnapSettings();
    ig::retained::FloatWindow* snapDialog() const;
    void setOrthogonal(bool orthogonal);
    void setGridVisible(bool visible);

    using AssetDropHandler = ct::Function<void(const ct::String&, const Math::Vec3&)>;

    void setAssetDropHandler(const AssetDropHandler& handler);

    using MaterialDropHandler = ct::Function<void(const ct::String&, Node*)>;

    void setMaterialDropHandler(const MaterialDropHandler& handler);
    void setScriptDropHandler(const MaterialDropHandler& handler);
    Node3D* pickAt(float screenX, float screenY) const;
    bool groundPoint(float x, float y, Math::Vec3& out) const;
    Math::Vec3 focusPoint() const;
    Math::Vec3 dropPosition(float screenX, float screenY) const;

    void setScenario(ScenarioId scenario);
    void setCameraAspectProvider(const ct::Function<float()>& provider);
    EditorCameraGizmo& cameraGizmo();
    EditorLightGizmo& lightGizmo();
    EditorShapeGizmo& shapeGizmo();
    EditorShapeHandles& shapeHandles();
    EditorNodeIcons& nodeIcons();
    TerrainEditTool& terrainTool();
    ScenarioId scenario() const;
    void setClearColor(const Math::Vec4& color);

    void connectPreDraw();
    void disconnectPreDraw();
    void update(float delta);
    void setSuspended(bool suspended);
    bool suspended() const;
    bool handleEvent(const InputEvent& event);

    EditorCamera& camera();
    const EditorCamera& camera() const;
    ViewportId viewportId() const;
    CameraId cameraId() const;
    TextureId texture() const;
    ig::retained::TextureHandle shownTexture() const;
    ig::retained::Widget* view() const;
    int orthogonalButton() const;
    ig::retained::Widget* column() const;

    std::uint32_t width() const;
    std::uint32_t height() const;
    unsigned int resizeCount() const;
    bool rendering() const;
    bool dragging() const;
    bool hovered(int x, int y) const;

private:
    struct Link
    {
        EditorViewportPanel* owner;
    };

    bool visibleInTree() const;
    void assetDropped(const ct::String& path, float x, float y);
    void syncSize();
    void applyCamera();
    void syncToolBar();
    void drawOverlays();
    void toggledButton(int id, bool checked);
    void clickedButton(int id);
    bool gizmoWants(const InputEvent& event) const;
    void syncMouseCapture();
    bool typing() const;
    bool wants(const InputEvent& event) const;

    RenderingServer& mServer;
    ct::Rc<Link> mLink;
    EditorCamera mCamera;
    CameraId mCameraId;
    ViewportId mViewport;
    ScenarioId mScenario;
    ViewportUpdateMode mUpdateMode;
    ViewportImage* mView;
    ig::retained::BoxLayout* mColumn;
    ig::retained::ToolBar* mToolBar;
    EditorGizmo* mGizmo;
    EditorGrid mGrid;
    ViewportRotationControl mRotation;
    EditorCameraGizmo mCameraGizmo;
    EditorLightGizmo mLightGizmo;
    EditorShapeGizmo mShapeGizmo;
    EditorNodeIcons mNodeIcons;
    ct::Function<float()> mAspectProvider;
    int mButtons[kButtonCount];
    bool mShowGizmos;
    TextureId mTexture;
    int mOrthogonalButton;
    std::uint32_t mWidth;
    std::uint32_t mHeight;
    unsigned int mResizes;
    unsigned int mDragButtons;
    bool mCaptured;
    int mMouseX;
    int mMouseY;
    int mSavedMouseX;
    int mSavedMouseY;
    bool mPreDraw;
    std::uint64_t mLastTime;
    bool mSyncing;
    bool mSuspended;
    AssetDropHandler mAssetDrop;
    MaterialDropHandler mMaterialDrop;
    MaterialDropHandler mScriptDrop;
    TerrainEditTool mTerrainTool;
    EditorShapeHandles mShapeHandles;
};

} // namespace prisma3d
