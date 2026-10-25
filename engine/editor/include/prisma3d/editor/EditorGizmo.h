#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorNodeIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include "mathc.h"

#include <ct/function.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class BatchRenderer;
class Node;
class Node3D;
class SceneTree;

class EditorGizmo : private SceneTreeListener
{
public:
    static constexpr float kArrowRadius = 0.065f;
    static constexpr float kArrowSize = 0.35f;
    static constexpr float kRingHalfWidth = 0.1f;
    static constexpr float kPlaneSize = 0.25f;
    static constexpr float kPlaneDst = 0.2f;
    static constexpr float kCircleSize = 1.1f;
    static constexpr float kScaleSize = 0.15f;
    static constexpr float kScaleOffset = kCircleSize + 0.3f;
    static constexpr float kArrowOffset = kCircleSize + 0.3f;
    static constexpr float kMaxZ = 1000000.0f;
    static constexpr float kMovementThreshold = 8.0f;

    static constexpr const char* kTranslateAction = "Translate";
    static constexpr const char* kRotateAction = "Rotate";
    static constexpr const char* kScaleAction = "Scale";

    static constexpr int kHighlightMove = 0;
    static constexpr int kHighlightRotate = 3;
    static constexpr int kHighlightMovePlane = 6;
    static constexpr int kHighlightScale = 9;
    static constexpr int kHighlightScalePlane = 12;

    enum class Tool : int
    {
        Transform,
        Move,
        Rotate,
        Scale,
        Select
    };

    enum class Mode : int
    {
        None,
        Rotate,
        Translate,
        Scale
    };

    enum class Plane : int
    {
        View,
        XAxis,
        YAxis,
        ZAxis,
        YZ,
        XZ,
        XY
    };

    struct Settings
    {
        float size = 80.0f;
        float opacity = 0.9f;
        float translateSnap = 1.0f;
        float rotateSnap = 15.0f;
        float scaleSnap = 10.0f;
        Math::Vec4 axisColor[3] = { Math::Vec4(0.96f, 0.20f, 0.32f, 1.0f),
            Math::Vec4(0.53f, 0.84f, 0.01f, 1.0f), Math::Vec4(0.16f, 0.55f, 0.96f, 1.0f) };
    };

    EditorGizmo(SceneTree& tree, EditorSelection& selection);
    ~EditorGizmo() override;
    EditorGizmo(const EditorGizmo&) = delete;
    EditorGizmo& operator=(const EditorGizmo&) = delete;

    void setChangeListener(EditorChangeListener* listener);
    void setRootProvider(const ct::Function<Node*()>& provider);
    Node* sceneRoot() const;
    const EditorSelection& selection() const;

    Settings& settings();
    const Settings& settings() const;

    void setTool(Tool tool);
    Tool tool() const;
    void setLocalSpace(bool local);
    bool localSpace() const;
    void setSnapEnabled(bool enabled);
    bool snapEnabled() const;
    void setShown(bool shown);
    bool shown() const;
    bool snapActive() const;
    float translateSnap() const;
    float rotateSnap() const;
    float scaleSnap() const;

    void setModifiers(unsigned modifiers);
    unsigned modifiers() const;

    void setView(const EditorView& view);
    const EditorView& view() const;
    void update();
    bool handleEvent(const InputEvent& event, float x, float y);
    void cancel();
    void commit();
    void draw(BatchRenderer& batch);

    bool visible() const;
    const Math::Mat4& transform() const;
    float scale() const;
    int highlight() const;
    Mode mode() const;
    Plane plane() const;
    bool editing() const;
    Node3D* pick(float x, float y) const;
    EditorNodeIcons::Settings& iconSettings();
    unsigned int editCount() const;

private:
    struct Target
    {
        Node3D* node = nullptr;
        Math::Mat4 original;
        Math::Mat4 originalLocal;
        Math::Vec3 position;
        Math::Vec3 rotation;
        Math::Vec3 scale;
    };

    struct Edit
    {
        Mode mode = Mode::None;
        Plane plane = Plane::View;
        Math::Mat4 original;
        Math::Vec3 center;
        Math::Vec3 clickRay;
        Math::Vec3 clickRayPos;
        float mouseX = 0.0f;
        float mouseY = 0.0f;
        float originalMouseX = 0.0f;
        float originalMouseY = 0.0f;
        bool originalLocal = false;
        Math::Vec3 initialClickVector;
        Math::Vec3 previousRotationVector;
        Math::Vec3 viewAxisLocal;
        double accumulatedRotationAngle = 0.0;
        double rotationAngle = 0.0;
    };

    struct MeshData
    {
        ct::Vector<Math::Vec3> positions;
        ct::Vector<Math::Vec3> normals;
        ct::Vector<std::uint32_t> indices;
    };

    void nodeRemoved(Node* node) override;

    void buildMeshes();
    void collectSelection();
    Math::Mat4 axisTransform(int axis) const;
    Math::Mat4 planeTransform(int axis) const;
    Math::Vec3 gizmoColumn(int axis) const;
    float screenCost(const Math::Vec3& center, float radius, const Math::Vec3& pos) const;
    bool selectGizmo(float x, float y, bool highlightOnly);
    bool selectMove(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly, float x,
            float y);
    bool selectRotate(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly, float x,
            float y);
    bool selectScale(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly, float x,
            float y);
    bool pickPlane(const Math::Mat4& gt, const Math::Vec3& rayPos, const Math::Vec3& ray,
            float& cost, int& axis);
    void computeEdit(float x, float y);
    void beginTransform(Mode mode, float x, float y);
    void finishTransform();
    void updateTransform(bool shift);
    void applyTransform(const Math::Vec3& motion, double snap);
    Math::Mat4 computeTransform(Mode mode, const Math::Mat4& original,
            const Math::Mat4& originalLocal, Math::Vec3 motion, double extra, bool local,
            bool orthogonal, bool viewAxis) const;
    void applyTo(Node3D* node, const Math::Mat4& transform, bool local);
    bool onLeftPress(float x, float y);
    bool onLeftRelease(float x, float y);
    bool onMotion(float x, float y, unsigned buttonMask);
    bool onKey(const InputEvent& event);
    void selectClicked();
    void clearSelected();
    bool hasSelection() const;
    void notifyEdited();
    void recordTransform();
    Math::Vec3 cameraNormal() const;
    void drawTriangles(BatchRenderer& batch, const MeshData& mesh, const Math::Mat4& model,
            const Math::Vec4& color, bool cull);
    void drawRing(BatchRenderer& batch, int axis, const Math::Vec4& color);
    static Math::Vec4 highlightColor(const Math::Vec4& color);

    SceneTree& mTree;
    EditorSelection& mSelection;
    EditorChangeListener* mListener;
    ct::Function<Node*()> mRootProvider;
    Settings mSettings;
    EditorNodeIcons::Settings mIconSettings;
    Tool mTool;
    bool mLocal;
    bool mSnap;
    bool mShown;
    unsigned mModifiers;
    EditorView mView;
    bool mVisible;
    Math::Mat4 mTransform;
    float mScale;
    int mHighlight;
    Edit mEdit;
    ct::Vector<Target> mTargets;
    Node3D* mClicked;
    bool mSelectionInProgress;
    bool mClickedWantsAppend;
    bool mEmptyClick;
    bool mLeftDown;
    unsigned int mEdits;
    MeshData mMove;
    MeshData mPlane;
    MeshData mScaleBox;
    MeshData mRing;
    ct::Vector<Math::Vec3> mScratch;
};

} // namespace prisma3d
