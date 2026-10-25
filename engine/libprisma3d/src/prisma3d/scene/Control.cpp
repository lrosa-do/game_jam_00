#include "prisma3d/scene/Control.h"

#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

namespace
{

const char* const kPresetNames = "Top Left,Top Right,Bottom Left,Bottom Right,Center Left,"
                                 "Center Top,Center Right,Center Bottom,Center,Left Wide,"
                                 "Top Wide,Right Wide,Bottom Wide,VCenter Wide,HCenter Wide,"
                                 "Full Rect";
const char* const kGrowNames = "Begin,End,Both";

bool leftBegins(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::TopLeft:
        case Control::LayoutPreset::BottomLeft:
        case Control::LayoutPreset::CenterLeft:
        case Control::LayoutPreset::TopWide:
        case Control::LayoutPreset::BottomWide:
        case Control::LayoutPreset::LeftWide:
        case Control::LayoutPreset::HCenterWide:
        case Control::LayoutPreset::FullRect:
            return true;
        default:
            return false;
    }
}

bool leftCenters(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::CenterTop:
        case Control::LayoutPreset::CenterBottom:
        case Control::LayoutPreset::Center:
        case Control::LayoutPreset::VCenterWide:
            return true;
        default:
            return false;
    }
}

bool topBegins(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::TopLeft:
        case Control::LayoutPreset::TopRight:
        case Control::LayoutPreset::CenterTop:
        case Control::LayoutPreset::LeftWide:
        case Control::LayoutPreset::RightWide:
        case Control::LayoutPreset::TopWide:
        case Control::LayoutPreset::VCenterWide:
        case Control::LayoutPreset::FullRect:
            return true;
        default:
            return false;
    }
}

bool topCenters(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::CenterLeft:
        case Control::LayoutPreset::CenterRight:
        case Control::LayoutPreset::Center:
        case Control::LayoutPreset::HCenterWide:
            return true;
        default:
            return false;
    }
}

bool rightBegins(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::TopLeft:
        case Control::LayoutPreset::BottomLeft:
        case Control::LayoutPreset::CenterLeft:
        case Control::LayoutPreset::LeftWide:
            return true;
        default:
            return false;
    }
}

bool bottomBegins(Control::LayoutPreset preset)
{
    switch (preset)
    {
        case Control::LayoutPreset::TopLeft:
        case Control::LayoutPreset::TopRight:
        case Control::LayoutPreset::CenterTop:
        case Control::LayoutPreset::TopWide:
            return true;
        default:
            return false;
    }
}

float anchorFor(bool begins, bool centers)
{
    return begins ? 0.0f : centers ? 0.5f : 1.0f;
}

} // namespace

ig::Color Control::guiColor(const Math::Vec4& color)
{
    const auto channel = [](float value)
    {
        const float scaled = value * 255.0f + 0.5f;
        return static_cast<std::uint8_t>(scaled < 0.0f ? 0.0f : scaled > 255.0f ? 255.0f : scaled);
    };
    return ig::Color(channel(color.x), channel(color.y), channel(color.z), channel(color.w));
}

Control::Control()
        : mHGrow(GrowDirection::End),
          mVGrow(GrowDirection::End),
          mPreset(kPresetCustom),
          mPosition(0.0f, 0.0f),
          mSize(0.0f, 0.0f),
          mCustomMinimumSize(0.0f, 0.0f),
          mLastParentSize(0.0f, 0.0f),
          mScale(1.0f, 1.0f),
          mPivotOffset(0.0f, 0.0f),
          mModulate(1.0f, 1.0f, 1.0f, 1.0f),
          mVisible(true)
{
    for (int i = 0; i < 4; ++i)
    {
        mAnchor[i] = 0.0f;
        mOffset[i] = 0.0f;
    }
}

Control::~Control()
{
}

Control* Control::getParentControl() const
{
    Node* parent = getParent();
    return parent ? parent->castTo<Control>() : nullptr;
}

Math::Vec2 Control::getParentSize() const
{
    if (const Control* parent = getParentControl()) return parent->getSize();
    if (const Gui* gui = Gui::singleton())
    {
        const ig::Rect rect = gui->hudRect();
        return Math::Vec2(rect.width, rect.height);
    }
    return Math::Vec2(0.0f, 0.0f);
}

void Control::setAnchor(Side side, float anchor, bool keepOffset, bool pushOppositeAnchor)
{
    const int index = static_cast<int>(side);
    const int opposite = (index + 2) % 4;
    const Math::Vec2 parentSize = getParentSize();
    const float range = (side == kSideLeft || side == kSideRight) ? parentSize.x : parentSize.y;
    const float previous = mOffset[index] + mAnchor[index] * range;
    const float previousOpposite = mOffset[opposite] + mAnchor[opposite] * range;

    mAnchor[index] = anchor;
    if (((side == kSideLeft || side == kSideTop) && mAnchor[index] > mAnchor[opposite]) ||
            ((side == kSideRight || side == kSideBottom) && mAnchor[index] < mAnchor[opposite]))
    {
        if (pushOppositeAnchor) mAnchor[opposite] = mAnchor[index];
        else
            mAnchor[index] = mAnchor[opposite];
    }
    if (!keepOffset)
    {
        mOffset[index] = previous - mAnchor[index] * range;
        if (pushOppositeAnchor) mOffset[opposite] = previousOpposite - mAnchor[opposite] * range;
    }
    mPreset = kPresetCustom;
    sizeChanged();
}

float Control::getAnchor(Side side) const
{
    return mAnchor[static_cast<int>(side)];
}

void Control::setOffset(Side side, float value)
{
    if (mOffset[static_cast<int>(side)] == value) return;
    mOffset[static_cast<int>(side)] = value;
    sizeChanged();
}

float Control::getOffset(Side side) const
{
    return mOffset[static_cast<int>(side)];
}

void Control::setAnchorsPreset(LayoutPreset preset, bool keepOffsets)
{
    const int kept = mPreset;
    setAnchor(kSideLeft, anchorFor(leftBegins(preset), leftCenters(preset)), keepOffsets);
    setAnchor(kSideTop, anchorFor(topBegins(preset), topCenters(preset)), keepOffsets);
    setAnchor(kSideRight, anchorFor(rightBegins(preset), leftCenters(preset)), keepOffsets);
    setAnchor(kSideBottom, anchorFor(bottomBegins(preset), topCenters(preset)), keepOffsets);
    mPreset = kept;
}

void Control::setOffsetsPreset(LayoutPreset preset, PresetMode mode, float margin)
{
    const Math::Vec2 minimum = getCombinedMinimumSize();
    Math::Vec2 size = mSize;
    if (mode == PresetMode::MinSize || mode == PresetMode::KeepHeight) size.x = minimum.x;
    if (mode == PresetMode::MinSize || mode == PresetMode::KeepWidth) size.y = minimum.y;
    const Math::Vec2 parent = getParentSize();

    if (leftBegins(preset)) mOffset[0] = parent.x * (0.0f - mAnchor[0]) + margin;
    else if (leftCenters(preset))
        mOffset[0] = parent.x * (0.5f - mAnchor[0]) - size.x * 0.5f;
    else
        mOffset[0] = parent.x * (1.0f - mAnchor[0]) - size.x - margin;

    if (topBegins(preset)) mOffset[1] = parent.y * (0.0f - mAnchor[1]) + margin;
    else if (topCenters(preset))
        mOffset[1] = parent.y * (0.5f - mAnchor[1]) - size.y * 0.5f;
    else
        mOffset[1] = parent.y * (1.0f - mAnchor[1]) - size.y - margin;

    if (rightBegins(preset)) mOffset[2] = parent.x * (0.0f - mAnchor[2]) + size.x + margin;
    else if (leftCenters(preset))
        mOffset[2] = parent.x * (0.5f - mAnchor[2]) + size.x * 0.5f;
    else
        mOffset[2] = parent.x * (1.0f - mAnchor[2]) - margin;

    if (bottomBegins(preset)) mOffset[3] = parent.y * (0.0f - mAnchor[3]) + size.y + margin;
    else if (topCenters(preset))
        mOffset[3] = parent.y * (0.5f - mAnchor[3]) + size.y * 0.5f;
    else
        mOffset[3] = parent.y * (1.0f - mAnchor[3]) - margin;

    sizeChanged();
}

void Control::setGrowDirectionPreset(LayoutPreset preset)
{
    switch (preset)
    {
        case LayoutPreset::TopLeft:
        case LayoutPreset::BottomLeft:
        case LayoutPreset::CenterLeft:
        case LayoutPreset::LeftWide:
            mHGrow = GrowDirection::End;
            break;
        case LayoutPreset::TopRight:
        case LayoutPreset::BottomRight:
        case LayoutPreset::CenterRight:
        case LayoutPreset::RightWide:
            mHGrow = GrowDirection::Begin;
            break;
        default:
            mHGrow = GrowDirection::Both;
            break;
    }
    switch (preset)
    {
        case LayoutPreset::TopLeft:
        case LayoutPreset::TopRight:
        case LayoutPreset::CenterTop:
        case LayoutPreset::TopWide:
            mVGrow = GrowDirection::End;
            break;
        case LayoutPreset::BottomLeft:
        case LayoutPreset::BottomRight:
        case LayoutPreset::CenterBottom:
        case LayoutPreset::BottomWide:
            mVGrow = GrowDirection::Begin;
            break;
        default:
            mVGrow = GrowDirection::Both;
            break;
    }
    sizeChanged();
}

void Control::setLayoutPreset(int preset)
{
    if (preset < 0 || preset > static_cast<int>(LayoutPreset::FullRect))
    {
        mPreset = kPresetCustom;
        return;
    }
    const LayoutPreset layout = static_cast<LayoutPreset>(preset);
    setAnchorsPreset(layout, true);
    PresetMode mode = PresetMode::KeepSize;
    switch (layout)
    {
        case LayoutPreset::LeftWide:
        case LayoutPreset::RightWide:
        case LayoutPreset::VCenterWide:
            mode = PresetMode::KeepWidth;
            break;
        case LayoutPreset::TopWide:
        case LayoutPreset::BottomWide:
        case LayoutPreset::HCenterWide:
            mode = PresetMode::KeepHeight;
            break;
        case LayoutPreset::FullRect:
            mode = PresetMode::MinSize;
            break;
        default:
            break;
    }
    setOffsetsPreset(layout, mode);
    setGrowDirectionPreset(layout);
    mPreset = preset;
}

int Control::getLayoutPreset() const
{
    return mPreset;
}

void Control::computeLayoutRect(const Math::Vec2& position, const Math::Vec2& size)
{
    const Math::Vec2 parent = getParentSize();
    const float leftGrow = mHGrow == GrowDirection::Begin ? 1.0f
                           : mHGrow == GrowDirection::End ? 0.0f
                                                          : 0.5f;
    const float topGrow = mVGrow == GrowDirection::Begin ? 1.0f
                          : mVGrow == GrowDirection::End ? 0.0f
                                                         : 0.5f;
    const float x = position.x - (size.x - mSize.x) * leftGrow;
    const float y = position.y - (size.y - mSize.y) * topGrow;
    mOffset[0] = x - mAnchor[0] * parent.x;
    mOffset[1] = y - mAnchor[1] * parent.y;
    mOffset[2] = x + size.x - mAnchor[2] * parent.x;
    mOffset[3] = y + size.y - mAnchor[3] * parent.y;
}

void Control::setPosition(const Math::Vec2& position)
{
    computeLayoutRect(position, mSize);
    sizeChanged();
}

Math::Vec2 Control::getPosition() const
{
    return mPosition;
}

void Control::setSize(const Math::Vec2& size)
{
    const Math::Vec2 minimum = getCombinedMinimumSize();
    const Math::Vec2 wanted(size.x < minimum.x ? minimum.x : size.x,
            size.y < minimum.y ? minimum.y : size.y);
    computeLayoutRect(mPosition, wanted);
    sizeChanged();
}

Math::Vec2 Control::getSize() const
{
    return mSize;
}

Math::Vec2 Control::toGlobal(const Math::Vec2& local) const
{
    const Math::Vec2 inParent(mPosition.x + mPivotOffset.x + (local.x - mPivotOffset.x) * mScale.x,
            mPosition.y + mPivotOffset.y + (local.y - mPivotOffset.y) * mScale.y);
    if (const Control* parent = getParentControl()) return parent->toGlobal(inParent);
    if (const Gui* gui = Gui::singleton())
    {
        const ig::Rect rect = gui->hudRect();
        return Math::Vec2(rect.x + inParent.x, rect.y + inParent.y);
    }
    return inParent;
}

Math::Vec2 Control::getGlobalPosition() const
{
    return toGlobal(Math::Vec2(0.0f, 0.0f));
}

Math::Vec2 Control::getGlobalScale() const
{
    Math::Vec2 scale = mScale;
    for (const Control* parent = getParentControl(); parent; parent = parent->getParentControl())
    {
        scale.x *= parent->mScale.x;
        scale.y *= parent->mScale.y;
    }
    return scale;
}

Math::Vec2 Control::getDrawSize() const
{
    const Math::Vec2 scale = getGlobalScale();
    return Math::Vec2(mSize.x * scale.x, mSize.y * scale.y);
}

float Control::drawScale() const
{
    return getGlobalScale().y;
}

void Control::setScale(const Math::Vec2& scale)
{
    mScale = scale;
}

Math::Vec2 Control::getScale() const
{
    return mScale;
}

void Control::setPivotOffset(const Math::Vec2& pivot)
{
    mPivotOffset = pivot;
}

Math::Vec2 Control::getPivotOffset() const
{
    return mPivotOffset;
}

void Control::setCustomMinimumSize(const Math::Vec2& size)
{
    mCustomMinimumSize = size;
    updateMinimumSize();
}

Math::Vec2 Control::getCustomMinimumSize() const
{
    return mCustomMinimumSize;
}

Math::Vec2 Control::getMinimumSize() const
{
    return Math::Vec2(0.0f, 0.0f);
}

Math::Vec2 Control::getCombinedMinimumSize() const
{
    const Math::Vec2 own = getMinimumSize();
    return Math::Vec2(own.x > mCustomMinimumSize.x ? own.x : mCustomMinimumSize.x,
            own.y > mCustomMinimumSize.y ? own.y : mCustomMinimumSize.y);
}

void Control::updateMinimumSize()
{
    sizeChanged();
}

void Control::setHGrowDirection(GrowDirection direction)
{
    mHGrow = direction;
    sizeChanged();
}

Control::GrowDirection Control::getHGrowDirection() const
{
    return mHGrow;
}

void Control::setVGrowDirection(GrowDirection direction)
{
    mVGrow = direction;
    sizeChanged();
}

Control::GrowDirection Control::getVGrowDirection() const
{
    return mVGrow;
}

void Control::setVisible(bool visible)
{
    mVisible = visible;
}

bool Control::isVisible() const
{
    return mVisible;
}

bool Control::isVisibleInTree() const
{
    for (const Control* control = this; control; control = control->getParentControl())
        if (!control->mVisible) return false;
    return true;
}

void Control::setModulate(const Math::Vec4& color)
{
    mModulate = color;
}

Math::Vec4 Control::getModulate() const
{
    return mModulate;
}

Math::Vec4 Control::getEffectiveModulate() const
{
    Math::Vec4 color = mModulate;
    for (const Control* parent = getParentControl(); parent; parent = parent->getParentControl())
    {
        color.x *= parent->mModulate.x;
        color.y *= parent->mModulate.y;
        color.z *= parent->mModulate.z;
        color.w *= parent->mModulate.w;
    }
    return color;
}

void Control::sizeChanged()
{
    const Math::Vec2 parent = getParentSize();
    float edge[4];
    for (int i = 0; i < 4; ++i) edge[i] = mOffset[i] + mAnchor[i] * ((i & 1) ? parent.y : parent.x);

    Math::Vec2 position(edge[0], edge[1]);
    Math::Vec2 size(edge[2] - edge[0], edge[3] - edge[1]);
    const Math::Vec2 minimum = getCombinedMinimumSize();
    if (minimum.x > size.x)
    {
        if (mHGrow == GrowDirection::Begin) position.x += size.x - minimum.x;
        else if (mHGrow == GrowDirection::Both)
            position.x += 0.5f * (size.x - minimum.x);
        size.x = minimum.x;
    }
    if (minimum.y > size.y)
    {
        if (mVGrow == GrowDirection::Begin) position.y += size.y - minimum.y;
        else if (mVGrow == GrowDirection::Both)
            position.y += 0.5f * (size.y - minimum.y);
        size.y = minimum.y;
    }
    mPosition = position;
    mSize = size;
    mLastParentSize = parent;

    for (std::size_t i = 0; i < getChildCount(); ++i)
        if (Control* child = getChild(i)->castTo<Control>()) child->sizeChanged();
}

void Control::draw(Gui& gui)
{
    (void) gui;
}

void Control::notificationHandler(int what)
{
    Node::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
        case kNotificationParented:
            sizeChanged();
            break;
        case kNotificationDraw:
        {
            if (!isVisibleInTree()) break;
            Gui* gui = Gui::singleton();
            if (!gui) break;
            if (!getParentControl())
            {
                const Math::Vec2 parent = getParentSize();
                if (parent.x != mLastParentSize.x || parent.y != mLastParentSize.y) sizeChanged();
            }
            draw(*gui);
            break;
        }
        default:
            break;
    }
}

void Control::setAnchorLeft(float value)
{
    setAnchor(kSideLeft, value);
}

float Control::getAnchorLeft() const
{
    return mAnchor[0];
}

void Control::setAnchorTop(float value)
{
    setAnchor(kSideTop, value);
}

float Control::getAnchorTop() const
{
    return mAnchor[1];
}

void Control::setAnchorRight(float value)
{
    setAnchor(kSideRight, value);
}

float Control::getAnchorRight() const
{
    return mAnchor[2];
}

void Control::setAnchorBottom(float value)
{
    setAnchor(kSideBottom, value);
}

float Control::getAnchorBottom() const
{
    return mAnchor[3];
}

void Control::setOffsetLeft(float value)
{
    setOffset(kSideLeft, value);
}

float Control::getOffsetLeft() const
{
    return mOffset[0];
}

void Control::setOffsetTop(float value)
{
    setOffset(kSideTop, value);
}

float Control::getOffsetTop() const
{
    return mOffset[1];
}

void Control::setOffsetRight(float value)
{
    setOffset(kSideRight, value);
}

float Control::getOffsetRight() const
{
    return mOffset[2];
}

void Control::setOffsetBottom(float value)
{
    setOffset(kSideBottom, value);
}

float Control::getOffsetBottom() const
{
    return mOffset[3];
}

void Control::bindProperties()
{
    using DB = NodeClassDB;
    using P = Control;
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "visible"), &P::setVisible, &P::isVisible);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Int, "anchors_preset", PropertyHint::Enum, kPresetNames),
            &P::setLayoutPreset, &P::getLayoutPreset);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "anchor_left", 0.0, 1.0, 0.01),
            &P::setAnchorLeft, &P::getAnchorLeft);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "anchor_top", 0.0, 1.0, 0.01),
            &P::setAnchorTop, &P::getAnchorTop);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "anchor_right", 0.0, 1.0, 0.01),
            &P::setAnchorRight, &P::getAnchorRight);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "anchor_bottom", 0.0, 1.0, 0.01),
            &P::setAnchorBottom, &P::getAnchorBottom);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "offset_left"), &P::setOffsetLeft,
            &P::getOffsetLeft);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "offset_top"), &P::setOffsetTop,
            &P::getOffsetTop);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "offset_right"), &P::setOffsetRight,
            &P::getOffsetRight);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "offset_bottom"), &P::setOffsetBottom,
            &P::getOffsetBottom);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Int, "grow_horizontal", PropertyHint::Enum, kGrowNames),
            &P::setHGrowDirection, &P::getHGrowDirection);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Int, "grow_vertical", PropertyHint::Enum, kGrowNames),
            &P::setVGrowDirection, &P::getVGrowDirection);
    // The position and the size follow from the anchors and the offsets, which are what a scene
    // stores; a scene that sets them still loads, they move the offsets.
    PropertyInfo positionInfo(PropertyType::Vec2, "position");
    positionInfo.usage = PropertyUsage::Editor;
    DB::addProperty<P>(positionInfo, &P::setPosition, &P::getPosition);
    PropertyInfo sizeInfo(PropertyType::Vec2, "size");
    sizeInfo.usage = PropertyUsage::Editor;
    DB::addProperty<P>(sizeInfo, &P::setSize, &P::getSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "custom_minimum_size"),
            &P::setCustomMinimumSize, &P::getCustomMinimumSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "modulate"), &P::setModulate,
            &P::getModulate);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "scale"), &P::setScale, &P::getScale);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "pivot_offset"), &P::setPivotOffset,
            &P::getPivotOffset);
}

} // namespace prisma3d
