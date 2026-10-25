#include "prisma3d/editor/ViewportRotationControl.h"

#include <igui/widgets/Theme.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

struct AxisColor
{
    float r;
    float g;
    float b;
};

const AxisColor kAxisColors[3] = { { 0.96f, 0.20f, 0.32f }, { 0.53f, 0.84f, 0.01f },
    { 0.16f, 0.55f, 0.96f } };

const EditorCamera::ViewType kMenuOptions[ViewportRotationControl::kAxisCount] = {
    EditorCamera::ViewType::Right, EditorCamera::ViewType::Top, EditorCamera::ViewType::Front,
    EditorCamera::ViewType::Left, EditorCamera::ViewType::Bottom, EditorCamera::ViewType::Rear
};

const float kMinimumAlpha = 0.35f;

unsigned char channel(float value)
{
    const float clamped = value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
    return static_cast<unsigned char>(clamped * 255.0f + 0.5f);
}

rt::Color toColor(float r, float g, float b, float a)
{
    return rt::Color(channel(r), channel(g), channel(b), channel(a));
}

float distance(const Math::Vec2& a, const Math::Vec2& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

Math::Vec2 limitLength(const Math::Vec2& value, float length)
{
    const float current = sqrtf(value.x * value.x + value.y * value.y);
    if (current > length && current > 0.0f) return value * (length / current);
    return value;
}

rt::Vec2 toVec(const Math::Vec2& value)
{
    return rt::Vec2{ value.x, value.y };
}

} // namespace

ViewportRotationControl::ViewportRotationControl(EditorCamera& camera)
        : mCamera(camera),
          mSavedCursor(),
          mOriginalMouse(0.0f, 0.0f),
          mViewWidth(0.0f),
          mViewHeight(0.0f),
          mOrbitingIndex(-1),
          mFocusedAxis(kNoFocus),
          mActivated(false),
          mOrbitStarted(false),
          mVisible(true)
{
}

void ViewportRotationControl::setViewSize(float width, float height)
{
    mViewWidth = width;
    mViewHeight = height;
}

Math::Vec2 ViewportRotationControl::origin() const
{
    return Math::Vec2(mViewWidth - kSize - kMargin, kMargin);
}

float ViewportRotationControl::size() const
{
    return kSize;
}

void ViewportRotationControl::setVisible(bool visible)
{
    mVisible = visible;
    if (!visible) cancel();
}

bool ViewportRotationControl::visible() const
{
    return mVisible;
}

bool ViewportRotationControl::fits() const
{
    const float room = kSize + kMargin * 2.0f;
    return mViewWidth >= room && mViewHeight >= room;
}

int ViewportRotationControl::focusedAxis() const
{
    return mFocusedAxis;
}

bool ViewportRotationControl::orbiting() const
{
    return mOrbitingIndex != -1;
}

bool ViewportRotationControl::activated() const
{
    return mActivated;
}

EditorCamera::ViewType ViewportRotationControl::viewOf(int axis)
{
    return axis >= 0 && axis < kAxisCount ? kMenuOptions[axis] : EditorCamera::ViewType::User;
}

Math::Vec2 ViewportRotationControl::toLocal(float x, float y) const
{
    const Math::Vec2 base = origin();
    return Math::Vec2(x - base.x, y - base.y);
}

bool ViewportRotationControl::insideCircle(const Math::Vec2& local) const
{
    return distance(local, Math::Vec2(kSize * 0.5f, kSize * 0.5f)) < kSize * 0.5f;
}

int ViewportRotationControl::sortedAxes(Axis2D* out) const
{
    const Math::Vec2 center(kSize * 0.5f, kSize * 0.5f);
    const float radius = kSize * 0.5f - kAxisCircleRadius - 2.0f;
    const Math::Mat3 basis = mCamera.toCameraTransform().UpperLeft3x3();

    int count = 0;
    for (int i = 0; i < 3; ++i)
    {
        const Math::Vec3 axis3d(basis.col0[i], basis.col1[i], basis.col2[i]);
        const Math::Vec2 axisVector(axis3d.x * radius, -axis3d.y * radius);

        if (fabsf(axis3d.z) < 1.0f)
        {
            Axis2D positive;
            positive.axis = i;
            positive.screenPoint = center + axisVector;
            positive.zAxis = axis3d.z;
            positive.positive = true;
            out[count++] = positive;

            Axis2D negative;
            negative.axis = i + 3;
            negative.screenPoint = center - axisVector;
            negative.zAxis = -axis3d.z;
            negative.positive = false;
            out[count++] = negative;
        }
        else
        {
            Axis2D aligned;
            aligned.axis = i + (axis3d.z <= 0.0f ? 0 : 3);
            aligned.screenPoint = center;
            aligned.zAxis = 1.0f;
            aligned.positive = axis3d.z > 0.0f;
            out[count++] = aligned;
        }
    }

    for (int i = 1; i < count; ++i)
    {
        const Axis2D value = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].zAxis > value.zAxis)
        {
            out[j + 1] = out[j];
            --j;
        }
        out[j + 1] = value;
    }
    return count;
}

void ViewportRotationControl::updateFocus(const Math::Vec2& local)
{
    mFocusedAxis = kNoFocus;
    if (insideCircle(local)) mFocusedAxis = kFocusBackground;

    Axis2D axes[kAxisCount];
    const int count = sortedAxes(axes);
    for (int i = 0; i < count; ++i)
        if (distance(local, axes[i].screenPoint) < kAxisCircleRadius) mFocusedAxis = axes[i].axis;
}

void ViewportRotationControl::cancel()
{
    if (mOrbitStarted)
    {
        mCamera.cursor() = mSavedCursor;
        mOrbitStarted = false;
    }
    mActivated = false;
}

void ViewportRotationControl::mouseExit()
{
    if (mOrbitingIndex == -1) mFocusedAxis = kNoFocus;
}

void ViewportRotationControl::processClick(const Math::Vec2& local, bool pressed)
{
    if (pressed)
    {
        if (insideCircle(local)) mOrbitingIndex = 100;
        return;
    }
    if (mFocusedAxis > -1 && mActivated)
    {
        mCamera.showView(viewOf(mFocusedAxis));
        updateFocus(local);
    }
    mOrbitingIndex = -1;
    mOrbitStarted = false;
}

void ViewportRotationControl::processDrag(const Math::Vec2& local, const Math::Vec2& relative)
{
    const bool thresholdPassed = distance(mOriginalMouse, local) > kDragThreshold;
    if (mOrbitingIndex == 100 && mActivated && thresholdPassed)
    {
        if (!mOrbitStarted)
        {
            mOrbitStarted = true;
            mSavedCursor = mCamera.cursor();
        }
        mCamera.cursorOrbit(relative);
        mFocusedAxis = kFocusBackground;
    }
    else
        updateFocus(local);
}

bool ViewportRotationControl::handleEvent(const InputEvent& event, float localX, float localY)
{
    if (!mVisible || !fits()) return false;
    const Math::Vec2 local = toLocal(localX, localY);

    switch (event.type)
    {
        case InputEventType::Key:
            if (event.key.pressed && event.key.key == Key::Escape && mOrbitStarted)
            {
                cancel();
                return true;
            }
            return false;
        case InputEventType::MouseButton:
            if (event.mouseButton.button == MouseButton::Left)
            {
                if (event.mouseButton.pressed)
                {
                    if (!insideCircle(local)) return false;
                    updateFocus(local);
                    processClick(local, true);
                    mActivated = true;
                    mOriginalMouse = local;
                    return true;
                }
                if (mOrbitingIndex == -1) return false;
                if (!mOrbitStarted) updateFocus(local);
                processClick(local, false);
                return true;
            }
            if (event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                    mOrbitStarted)
            {
                cancel();
                return true;
            }
            return false;
        case InputEventType::MouseMotion:
            processDrag(local, Math::Vec2(static_cast<float>(event.mouseMotion.relativeX),
                                       static_cast<float>(event.mouseMotion.relativeY)));
            return mOrbitingIndex != -1;
        default:
            return false;
    }
}

void ViewportRotationControl::drawAxis(rt::PaintContext& ctx, const Axis2D& axis,
        const Math::Vec2& base) const
{
    const bool focused = mFocusedAxis == axis.axis;
    const int direction = axis.axis % 3;
    const AxisColor color = kAxisColors[direction];
    const float alpha = focused ? 1.0f
                                : kMinimumAlpha + (1.0f - kMinimumAlpha) *
                                                          (((axis.zAxis + 1.0f) * 0.5f) / 0.5f);

    float lineR = color.r;
    float lineG = color.g;
    float lineB = color.b;
    float lineA = alpha;
    if (focused)
    {
        lineR = color.r + (1.0f - color.r) * 0.25f;
        lineG = color.g + (1.0f - color.g) * 0.25f;
        lineB = color.b + (1.0f - color.b) * 0.25f;
        lineA = 1.0f;
    }
    const rt::Color fill = toColor(lineR, lineG, lineB, lineA);
    const Math::Vec2 point = base + axis.screenPoint;
    const rt::Theme& theme = rt::Theme::instance();

    ctx.font.SetFontSize(theme.fontSize);
    ctx.font.SetBatch(&ctx.text);
    const float ascender = ctx.font.GetAscender();

    if (axis.positive)
    {
        const Math::Vec2 center = base + Math::Vec2(kSize * 0.5f, kSize * 0.5f);
        const Math::Vec2 diff = point - center;
        const float length = sqrtf(diff.x * diff.x + diff.y * diff.y);
        const float lineLength =
                length - kAxisCircleRadius - 0.5f > 0.0f ? length - kAxisCircleRadius - 0.5f : 0.0f;
        ctx.drawList.addLine(toVec(center + limitLength(diff, 0.5f)),
                toVec(center + limitLength(diff, lineLength)), fill, 1.5f);
        ctx.drawList.addCircleFilled(toVec(point), kAxisCircleRadius, fill, 24);

        const char* name = direction == 0 ? "X" : (direction == 1 ? "Y" : "Z");
        const rt::Color text = focused ? toColor(1.0f, 1.0f, 1.0f, alpha)
                                       : toColor(0.0f, 0.0f, 0.0f, alpha * 0.6f);
        ctx.font.SetColor(text);
        const float width = ctx.font.GetTextWidth(name);
        ctx.font.Print(name, point.x - width * 0.5f, point.y + ascender * 0.35f);
    }
    else
    {
        ctx.drawList.addCircleFilled(toVec(point), kAxisCircleRadius, fill, 24);
        const rt::Color inner = toColor(lineR * 0.6f, lineG * 0.6f, lineB * 0.6f, lineA);
        ctx.drawList.addCircleFilled(toVec(point), kAxisCircleRadius * 0.8f, inner, 24);

        const char* name = direction == 0 ? "-X" : (direction == 1 ? "-Y" : "-Z");
        const rt::Color text = focused ? toColor(1.0f, 1.0f, 1.0f, alpha)
                                       : toColor(color.r, color.g, color.b, 0.0f);
        ctx.font.SetColor(text);
        const float width = ctx.font.GetTextWidth(name);
        ctx.font.Print(name, point.x - width * 0.5f, point.y + ascender * 0.35f);
    }
}

void ViewportRotationControl::draw(rt::PaintContext& ctx, float viewX, float viewY) const
{
    if (!mVisible || !fits()) return;
    const Math::Vec2 base = origin() + Math::Vec2(viewX, viewY);

    if (mFocusedAxis > kNoFocus || mOrbitingIndex != -1)
        ctx.drawList.addCircleFilled(toVec(base + Math::Vec2(kSize * 0.5f, kSize * 0.5f)),
                kSize * 0.5f, toColor(0.5f, 0.5f, 0.5f, 0.25f), 48);

    Axis2D axes[kAxisCount];
    const int count = sortedAxes(axes);
    for (int i = 0; i < count; ++i) drawAxis(ctx, axes[i], base);
}

} // namespace prisma3d
