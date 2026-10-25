#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorCamera.h"

#include <igui/widgets/Widget.hpp>

#include "mathc.h"

namespace prisma3d
{

class ViewportRotationControl
{
public:
    static constexpr float kSize = 80.0f;
    static constexpr float kMargin = 10.0f;
    static constexpr float kAxisCircleRadius = 8.0f;
    static constexpr float kDragThreshold = 4.0f;
    static constexpr int kAxisCount = 6;
    static constexpr int kNoFocus = -2;
    static constexpr int kFocusBackground = -1;

    struct Axis2D
    {
        Math::Vec2 screenPoint;
        float zAxis = -99.0f;
        int axis = -1;
        bool positive = true;
    };

    explicit ViewportRotationControl(EditorCamera& camera);

    void setViewSize(float width, float height);
    Math::Vec2 origin() const;
    float size() const;
    void setVisible(bool visible);
    bool visible() const;
    bool fits() const;

    int sortedAxes(Axis2D* out) const;
    static EditorCamera::ViewType viewOf(int axis);

    bool handleEvent(const InputEvent& event, float localX, float localY);
    void cancel();
    void mouseExit();

    int focusedAxis() const;
    bool orbiting() const;
    bool activated() const;

    void draw(ig::retained::PaintContext& ctx, float viewX, float viewY) const;

private:
    void updateFocus(const Math::Vec2& local);
    bool insideCircle(const Math::Vec2& local) const;
    Math::Vec2 toLocal(float x, float y) const;
    void processClick(const Math::Vec2& local, bool pressed);
    void processDrag(const Math::Vec2& local, const Math::Vec2& relative);
    void drawAxis(ig::retained::PaintContext& ctx, const Axis2D& axis,
            const Math::Vec2& base) const;

    EditorCamera& mCamera;
    EditorCamera::Cursor mSavedCursor;
    Math::Vec2 mOriginalMouse;
    float mViewWidth;
    float mViewHeight;
    int mOrbitingIndex;
    int mFocusedAxis;
    bool mActivated;
    bool mOrbitStarted;
    bool mVisible;
};

} // namespace prisma3d
