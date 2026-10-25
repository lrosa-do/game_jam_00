#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class CollisionShape3D;
class EditorSelection;

class EditorShapeHandles
{
public:
    static constexpr float kPickRadius = 10.0f;
    static constexpr float kHandleSize = 5.0f;
    static constexpr float kMinimum = 0.001f;

    struct Handle
    {
        Math::Vec3 position;
        Math::Vec3 axis;
        const char* property = nullptr;
        int component = -1;
        bool half = false;
    };

    EditorShapeHandles();

    void setChangeListener(EditorChangeListener* listener);
    void setSelection(const EditorSelection* selection);

    CollisionShape3D* target() const;
    std::size_t handleCount() const;
    const Handle& handle(std::size_t index) const;
    int hovered() const;
    bool dragging() const;
    unsigned int editCount() const;

    bool handleEvent(const InputEvent& event, float x, float y, bool navigating);
    void update(const EditorView& view);
    void draw(BatchRenderer& batch);

private:
    void refreshTarget();
    void collectHandles();
    int pick(float x, float y) const;
    bool axisPoint(const Math::Vec3& origin, const Math::Vec3& direction, float& along) const;
    void beginDrag(int index);
    void drag();
    void endDrag(bool commit);
    PropertyValue currentValue() const;
    void applyValue(float along);

    EditorChangeListener* mListener;
    const EditorSelection* mSelection;
    CollisionShape3D* mTarget;
    EditorView mView;
    ct::Vector<Handle> mHandles;
    int mHovered;
    int mActive;
    float mMouseX;
    float mMouseY;
    float mGrabOffset;
    PropertyValue mBefore;
    unsigned int mEdits;
};

} // namespace prisma3d
