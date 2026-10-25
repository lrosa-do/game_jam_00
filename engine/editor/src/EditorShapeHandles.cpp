#include "prisma3d/editor/EditorShapeHandles.h"

#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <math.h>

namespace prisma3d
{

using Math::Vec3;

namespace
{

const float kColor[3] = { 1.0f, 0.85f, 0.3f };
const float kHoverColor[3] = { 1.0f, 1.0f, 1.0f };

} // namespace

EditorShapeHandles::EditorShapeHandles()
        : mListener(nullptr),
          mSelection(nullptr),
          mTarget(nullptr),
          mHovered(-1),
          mActive(-1),
          mMouseX(0.0f),
          mMouseY(0.0f),
          mGrabOffset(0.0f),
          mEdits(0)
{
}

void EditorShapeHandles::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
}

void EditorShapeHandles::setSelection(const EditorSelection* selection)
{
    mSelection = selection;
}

CollisionShape3D* EditorShapeHandles::target() const
{
    return mTarget;
}

std::size_t EditorShapeHandles::handleCount() const
{
    return mHandles.size();
}

const EditorShapeHandles::Handle& EditorShapeHandles::handle(std::size_t index) const
{
    return mHandles[index];
}

int EditorShapeHandles::hovered() const
{
    return mHovered;
}

bool EditorShapeHandles::dragging() const
{
    return mActive >= 0;
}

unsigned int EditorShapeHandles::editCount() const
{
    return mEdits;
}

void EditorShapeHandles::refreshTarget()
{
    CollisionShape3D* found = nullptr;
    if (mSelection)
    {
        const ct::Vector<Node*>& nodes = mSelection->selectedNodes();
        if (nodes.size() == 1) found = nodes[0]->castTo<CollisionShape3D>();
    }
    if (found && !found->isVisibleInTree()) found = nullptr;
    if (found != mTarget)
    {
        if (mActive >= 0) endDrag(false);
        mTarget = found;
        mHovered = -1;
    }
}

void EditorShapeHandles::collectHandles()
{
    mHandles.clear();
    if (!mTarget) return;
    const Math::Mat4 global = mTarget->getGlobalTransform();
    const Vec3 axes[3] = { global[0].xyz().Normalized(), global[1].xyz().Normalized(),
        global[2].xyz().Normalized() };
    const Vec3 origin = global.GetTranslation();
    const auto add =
            [&](const Vec3& axis, float distance, const char* property, int component, bool half)
    {
        Handle handle;
        handle.axis = axis;
        handle.position = origin + axis * distance;
        handle.property = property;
        handle.component = component;
        handle.half = half;
        mHandles.push_back(handle);
    };
    switch (mTarget->getKind())
    {
        case CollisionShapeKind::Box:
        {
            const Vec3 half = mTarget->getBoxSize() * 0.5f;
            const float extents[3] = { half.x, half.y, half.z };
            for (int a = 0; a < 3; ++a)
            {
                add(axes[a], extents[a], "box_size", a, true);
                add(axes[a] * -1.0f, extents[a], "box_size", a, true);
            }
            break;
        }
        case CollisionShapeKind::Sphere:
            add(axes[0], mTarget->getSphereRadius(), "sphere_radius", -1, false);
            add(axes[2], mTarget->getSphereRadius(), "sphere_radius", -1, false);
            break;
        case CollisionShapeKind::Capsule:
            add(axes[0], mTarget->getCapsuleRadius(), "capsule_radius", -1, false);
            add(axes[1], mTarget->getCapsuleHeight() * 0.5f, "capsule_height", -1, true);
            add(axes[1] * -1.0f, mTarget->getCapsuleHeight() * 0.5f, "capsule_height", -1, true);
            break;
        case CollisionShapeKind::Cylinder:
            add(axes[0], mTarget->getCylinderRadius(), "cylinder_radius", -1, false);
            add(axes[1], mTarget->getCylinderHeight() * 0.5f, "cylinder_height", -1, true);
            add(axes[1] * -1.0f, mTarget->getCylinderHeight() * 0.5f, "cylinder_height", -1, true);
            break;
        default:
            break;
    }
}

int EditorShapeHandles::pick(float x, float y) const
{
    int best = -1;
    float bestDistance = kPickRadius;
    for (std::size_t i = 0; i < mHandles.size(); ++i)
    {
        if (mView.behind(mHandles[i].position)) continue;
        const Math::Vec2 screen = mView.project(mHandles[i].position);
        const float dx = screen.x - x;
        const float dy = screen.y - y;
        const float distance = sqrtf(dx * dx + dy * dy);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            best = static_cast<int>(i);
        }
    }
    return best;
}

bool EditorShapeHandles::axisPoint(const Vec3& origin, const Vec3& direction, float& along) const
{
    const Vec3 rayOrigin = mView.rayOrigin(mMouseX, mMouseY);
    const Vec3 rayDirection = mView.rayDirection(mMouseX, mMouseY);
    const Vec3 w = origin - rayOrigin;
    const float a = direction.Dot(direction);
    const float b = direction.Dot(rayDirection);
    const float c = rayDirection.Dot(rayDirection);
    const float d = direction.Dot(w);
    const float e = rayDirection.Dot(w);
    const float denominator = a * c - b * b;
    if (fabsf(denominator) < 1.0e-6f) return false;
    along = (b * e - c * d) / denominator;
    return true;
}

PropertyValue EditorShapeHandles::currentValue() const
{
    PropertyValue value;
    if (mTarget && mActive >= 0)
        NodeClassDB::getProperty(mTarget, mHandles[static_cast<std::size_t>(mActive)].property,
                value);
    return value;
}

void EditorShapeHandles::beginDrag(int index)
{
    mActive = index;
    mBefore = currentValue();
    const Handle& handle = mHandles[static_cast<std::size_t>(index)];
    const Vec3 origin = mTarget->getGlobalPosition();
    float along = 0.0f;
    if (!axisPoint(origin, handle.axis, along)) along = 0.0f;
    const float current = (handle.position - origin).Dot(handle.axis);
    mGrabOffset = current - along;
}

void EditorShapeHandles::applyValue(float along)
{
    const Handle& handle = mHandles[static_cast<std::size_t>(mActive)];
    float extent = along + mGrabOffset;
    if (extent < kMinimum) extent = kMinimum;
    const float value = handle.half ? extent * 2.0f : extent;
    PropertyValue next;
    if (handle.component >= 0)
    {
        Vec3 size = mBefore.asVec3();
        if (handle.component == 0) size.x = value;
        else if (handle.component == 1)
            size.y = value;
        else
            size.z = value;
        next = PropertyValue::fromVec3(size);
    }
    else
        next = PropertyValue::fromFloat(value);
    NodeClassDB::setProperty(mTarget, handle.property, next);
}

void EditorShapeHandles::drag()
{
    if (mActive < 0 || !mTarget) return;
    const Handle& handle = mHandles[static_cast<std::size_t>(mActive)];
    float along = 0.0f;
    if (!axisPoint(mTarget->getGlobalPosition(), handle.axis, along)) return;
    applyValue(along);
}

void EditorShapeHandles::endDrag(bool commit)
{
    if (mActive < 0) return;
    const int index = mActive;
    mActive = -1;
    if (!mTarget) return;
    const char* property = mHandles[static_cast<std::size_t>(index)].property;
    PropertyValue after;
    NodeClassDB::getProperty(mTarget, property, after);
    if (!commit || after == mBefore)
    {
        NodeClassDB::setProperty(mTarget, property, mBefore);
        return;
    }
    NodeClassDB::setProperty(mTarget, property, mBefore);
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (undo)
        EditorTreeEdits::setProperty(undo, ct::String("Resize Shape"), mTarget,
                ct::String(property), mBefore, after);
    else
    {
        NodeClassDB::setProperty(mTarget, property, after);
        if (mListener) mListener->sceneEdited();
    }
    ++mEdits;
}

bool EditorShapeHandles::handleEvent(const InputEvent& event, float x, float y, bool navigating)
{
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            mMouseX = x;
            mMouseY = y;
            if (mActive >= 0)
            {
                drag();
                return true;
            }
            return false;
        case InputEventType::MouseButton:
            mMouseX = x;
            mMouseY = y;
            if (event.mouseButton.button == MouseButton::Left)
            {
                if (event.mouseButton.pressed)
                {
                    if (navigating || !mTarget || mHandles.empty()) return false;
                    const int index = pick(x, y);
                    if (index < 0) return false;
                    beginDrag(index);
                    return true;
                }
                if (mActive < 0) return false;
                endDrag(true);
                return true;
            }
            if (event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                    mActive >= 0)
            {
                endDrag(false);
                return true;
            }
            return false;
        case InputEventType::Key:
            if (event.key.pressed && event.key.key == Key::Escape && mActive >= 0)
            {
                endDrag(false);
                return true;
            }
            return false;
        default:
            return false;
    }
}

void EditorShapeHandles::update(const EditorView& view)
{
    mView = view;
    refreshTarget();
    if (mActive < 0) collectHandles();
    mHovered = mActive >= 0 ? mActive : pick(mMouseX, mMouseY);
}

void EditorShapeHandles::draw(BatchRenderer& batch)
{
    if (!mTarget || mHandles.empty()) return;
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

    const Vec3 right = mView.right();
    const Vec3 up = mView.up();
    const Vec3 eye = mView.position();
    for (std::size_t i = 0; i < mHandles.size(); ++i)
    {
        const Vec3& p = mHandles[i].position;
        float scale = kHandleSize;
        if (!mView.orthogonal())
        {
            const float distance = (p - eye).Length();
            const float pixels =
                    mView.height / (2.0f * tanf(mView.projection.fov * 0.5f * 0.01745329252f));
            scale = kHandleSize * distance / (pixels > 1.0f ? pixels : 1.0f);
        }
        else
            scale = kHandleSize * mView.projection.size / mView.height;
        const bool hot = static_cast<int>(i) == mHovered;
        const float* color = hot ? kHoverColor : kColor;
        batch.setColor(color[0], color[1], color[2], 1.0f);
        batch.begin(BatchRenderer::ModeTriangles);
        const Vec3 a = p - right * scale - up * scale;
        const Vec3 b = p + right * scale - up * scale;
        const Vec3 c = p + right * scale + up * scale;
        const Vec3 d = p - right * scale + up * scale;
        batch.vertex3(a.x, a.y, a.z);
        batch.vertex3(b.x, b.y, b.z);
        batch.vertex3(c.x, c.y, c.z);
        batch.vertex3(a.x, a.y, a.z);
        batch.vertex3(c.x, c.y, c.z);
        batch.vertex3(d.x, d.y, d.z);
        batch.end();
    }

    batch.popMatrix();
    batch.setSpace(space);
}

} // namespace prisma3d
