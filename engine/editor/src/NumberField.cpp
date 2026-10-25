#include "prisma3d/editor/NumberField.h"

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kDragThreshold = 4.0f;
const float kTagInset = 6.0f;
const float kFieldWidth = 72.0f;
const float kFieldHeight = 26.0f;
const float kGrabberWidth = 4.0f;
const float kGrabberRadius = 4.0f;
const float kGrabberHit = 9.0f;
const float kTrackHeight = 2.0f;
const float kShiftScale = 0.1f;

} // namespace

class NumberField::Editor : public rt::TextInput
{
public:
    IG_WIDGET_CLASS(Editor, rt::TextInput);

    explicit Editor(NumberField& owner)
            : rt::TextInput("", rt::TextInput::Mode::NumberOnly),
              mOwner(owner)
    {
    }

    void onKeyPress(rt::KeyEvent& event) override
    {
        if (event.key == rt::Key::Escape)
        {
            mOwner.cancelEdit();
            event.consumed = true;
            return;
        }
        rt::TextInput::onKeyPress(event);
    }

    void onFocusLost() override
    {
        mOwner.commitEdit();
    }

private:
    NumberField& mOwner;
};

NumberField::NumberField(const char* tag)
        : mTag(tag ? tag : ""),
          mTagColor(rt::Theme::instance().textDisabled),
          mEditor(nullptr),
          mValue(0.0f),
          mSensitivity(0.1f),
          mPressX(0.0f),
          mPressValue(0.0f),
          mMin(0.0),
          mMax(0.0),
          mGrabFrom(0.0f),
          mGrabRatio(0.0f),
          mRanged(false),
          mOrLess(false),
          mOrGreater(false),
          mGrabbing(false),
          mOverGrabber(false),
          mPressed(false),
          mDragged(false),
          mEditing(false)
{
    mText = format(0.0f);
    cursor_ = rt::CursorType::SizeH;
    mEditor = createChild<Editor>(*this);
    mEditor->setVisible(false);
    mEditor->submitted.connect([this](const ct::String&) { commitEdit(); });
    hoverLeave.connect(
            [this]
            {
                if (mOverGrabber)
                {
                    mOverGrabber = false;
                    markDirty();
                }
            });
}

NumberField::~NumberField()
{
}

float NumberField::value() const
{
    return mValue;
}

void NumberField::setValue(float value)
{
    setValueInternal(value, false);
}

void NumberField::setDragSensitivity(float unitsPerPixel)
{
    mSensitivity = unitsPerPixel;
}

float NumberField::dragSensitivity() const
{
    return mSensitivity;
}

void NumberField::setRange(double minimum, double maximum, bool orLess, bool orGreater)
{
    mRanged = maximum > minimum;
    mOrLess = orLess;
    mOrGreater = orGreater;
    mMin = minimum;
    mMax = maximum;
    markDirty();
}

void NumberField::clearRange()
{
    mRanged = false;
    markDirty();
}

bool NumberField::ranged() const
{
    return mRanged;
}

double NumberField::rangeMin() const
{
    return mMin;
}

double NumberField::rangeMax() const
{
    return mMax;
}

float NumberField::ratio() const
{
    if (!mRanged) return 0.0f;
    const double r = (static_cast<double>(mValue) - mMin) / (mMax - mMin);
    return r < 0.0 ? 0.0f : (r > 1.0 ? 1.0f : static_cast<float>(r));
}

void NumberField::setRatio(float ratio)
{
    if (!mRanged) return;
    const float clamped = ratio < 0.0f ? 0.0f : (ratio > 1.0f ? 1.0f : ratio);
    setValueInternal(static_cast<float>(mMin + (mMax - mMin) * static_cast<double>(clamped)), true);
}

bool NumberField::isGrabbing() const
{
    return mGrabbing;
}

float NumberField::trackOffset() const
{
    return kTagInset;
}

float NumberField::trackWidth() const
{
    const float width = rect_.w - kTagInset * 2.0f - kGrabberWidth;
    return width > 1.0f ? width : 1.0f;
}

bool NumberField::grabberVisible() const
{
    return mRanged && !mEditing && enabled_ && (mGrabbing || mOverGrabber || isHovered());
}

rt::Rect NumberField::grabberRect() const
{
    const rt::Rect abs = absoluteRect();
    const float x = abs.x + trackOffset() + ratio() * trackWidth();
    const float y = abs.y + abs.h - kTrackHeight - 6.0f;
    return { x, y, kGrabberWidth, kGrabberWidth };
}

bool NumberField::overGrabber(float localX, float localY) const
{
    if (!mRanged || mEditing) return false;
    const rt::Rect abs = absoluteRect();
    const rt::Rect grabber = grabberRect();
    const float cx = grabber.x + grabber.w * 0.5f - abs.x;
    const float cy = grabber.y + grabber.h * 0.5f - abs.y;
    const float dx = localX - cx;
    const float dy = localY - cy;
    return fabsf(dx) <= kGrabberHit && fabsf(dy) <= kGrabberHit;
}

void NumberField::setTag(const char* tag)
{
    mTag = tag ? tag : "";
    markDirty();
}

const ct::String& NumberField::tag() const
{
    return mTag;
}

void NumberField::setTagColor(const rt::Color& color)
{
    mTagColor = color;
    markDirty();
}

const ct::String& NumberField::text() const
{
    return mText;
}

ct::String NumberField::format(float value) const
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.4f", static_cast<double>(value));
    char* dot = strchr(buffer, '.');
    if (dot)
    {
        char* end = buffer + strlen(buffer);
        while (end > dot + 1 && end[-1] == '0') --end;
        if (end == dot + 1) --end;
        *end = '\0';
    }
    if (strcmp(buffer, "-0") == 0) return ct::String("0");
    return ct::String(buffer);
}

bool NumberField::isEditing() const
{
    return mEditing;
}

bool NumberField::isDragging() const
{
    return mPressed && mDragged;
}

bool NumberField::isActive() const
{
    return mPressed || mEditing;
}

rt::TextInput* NumberField::editor() const
{
    return mEditor;
}

void NumberField::beginEdit()
{
    if (mEditing || !enabled_) return;
    mEditing = true;
    mEditStart = format(mValue);
    mEditor->setText(mEditStart);
    mEditor->setVisible(true);
    mEditor->selectAll();
    rt::WidgetApp::instance().setFocused(mEditor);
    markDirty();
}

void NumberField::commitEdit()
{
    finishEdit(true);
}

void NumberField::cancelEdit()
{
    finishEdit(false);
}

void NumberField::finishEdit(bool commit)
{
    if (!mEditing) return;
    mEditing = false;
    const ct::String text = mEditor->text();
    mEditor->setVisible(false);
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (app && app->focused() == mEditor) app->setFocused(nullptr);
    markDirty();
    if (commit && text != mEditStart && !text.empty())
    {
        char* end = nullptr;
        const float parsed = strtof(text.c_str(), &end);
        if (end != text.c_str() && *end == '\0' && isfinite(parsed)) setValueInternal(parsed, true);
    }
    editFinished.emit();
}

void NumberField::setValueInternal(float value, bool notify)
{
    if (!isfinite(value) || value == mValue) return;
    mValue = value;
    mText = format(value);
    markDirty();
    if (notify) valueChanged.emit(value);
}

rt::Widget::Vec2f NumberField::sizeHint() const
{
    return { kFieldWidth, kFieldHeight };
}

void NumberField::layout()
{
    mEditor->setRect({ 0.0f, 0.0f, rect_.w, rect_.h });
    mEditor->layout();
}

void NumberField::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    rt::Rect clipped;
    if (ctx.clipRectIntersect(abs, clipped))
    {
        const rt::Color& background = isHovered() ? theme.inputBgHover : theme.inputBg;
        ctx.fill.SetColor(background);
        ctx.fill.RoundedRectangle(clipped.x, clipped.y, clipped.w, clipped.h, theme.borderRadius, 6,
                true);
    }

    if (!mEditing)
    {
        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.pushClip(abs);
        ctx.font.SetFontSize(theme.fontSize);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        const float baseline = abs.y + (abs.h + ascender) * 0.5f;
        float left = abs.x + kTagInset;
        if (!mTag.empty())
        {
            ctx.font.SetColor(mTagColor);
            ctx.font.Print(mTag.c_str(), left, baseline);
            left += ctx.font.GetTextWidth(mTag.c_str()) + kTagInset;
        }
        const float width = ctx.font.GetTextWidth(mText.c_str());
        float x = abs.x + abs.w - kTagInset - width;
        if (x < left) x = left;
        ctx.font.SetColor(enabled_ ? theme.textColor : theme.textDisabled);
        ctx.font.Print(mText.c_str(), x, baseline);
        ctx.popClip();
        if (overrideFont_) ctx.popFont();
    }

    if (mRanged && !mEditing)
    {
        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.font.SetFontSize(theme.fontSize);
        const float ascender = ctx.font.GetAscender();
        if (overrideFont_) ctx.popFont();
        const float baseline = (abs.h + ascender) * 0.5f;
        const float trackY = abs.y + (abs.h + baseline) * 0.5f - 1.0f;
        const float left = abs.x + trackOffset();
        const float width = trackWidth();
        const float filled = ratio() * width;
        rt::Color line = enabled_ ? theme.textColor : theme.textDisabled;
        ctx.pushClip(abs);
        line.a = static_cast<unsigned char>(51);
        ctx.fill.SetColor(line);
        ctx.fill.Rectangle(left, trackY + 1.0f, width, kTrackHeight, true);
        line.a = static_cast<unsigned char>(115);
        ctx.fill.SetColor(line);
        ctx.fill.Rectangle(left, trackY + 1.0f, filled, kTrackHeight, true);
        line.a = static_cast<unsigned char>(230);
        ctx.fill.SetColor(line);
        ctx.fill.Rectangle(left + filled, trackY, kGrabberWidth, kTrackHeight * 2.0f, true);
        if (grabberVisible())
        {
            const bool lit = mGrabbing || mOverGrabber;
            rt::Color handle = lit ? theme.focusColor : theme.textColor;
            handle.a = static_cast<unsigned char>(lit ? 255 : 200);
            ctx.fill.SetColor(handle);
            ctx.fillCircle(left + filled + kGrabberWidth * 0.5f, trackY + 1.0f, kGrabberRadius);
        }
        ctx.popClip();
    }

    if (ctx.clipRectIntersect(abs, clipped))
    {
        const rt::Color& border =
                mEditing ? theme.focusColor
                         : (isHovered() ? theme.inputBorderHover : theme.inputBorder);
        ctx.line.SetColor(border);
        ctx.line.RoundedRectangle(clipped.x, clipped.y, clipped.w, clipped.h, theme.borderRadius, 6,
                false);
    }

    rt::Widget::paint(ctx);
}

void NumberField::cancelDrag()
{
    if (!mPressed && !mGrabbing) return;
    const bool was = mDragged || mGrabbing;
    mPressed = false;
    mDragged = false;
    mGrabbing = false;
    if (was) setValueInternal(mPressValue, true);
    markDirty();
}

void NumberField::onMousePress(rt::MouseEvent& event)
{
    if (!enabled_) return;
    if (event.button == 1)
    {
        if (mPressed || mGrabbing)
        {
            cancelDrag();
            event.consumed = true;
        }
        return;
    }
    if (event.button != 0) return;
    mPressed = true;
    mDragged = false;
    mPressX = event.x;
    mPressValue = mValue;
    if (overGrabber(event.localX, event.localY))
    {
        mGrabbing = true;
        mGrabFrom = event.x;
        mGrabRatio = ratio();
    }
    event.consumed = true;
}

void NumberField::onMouseRelease(rt::MouseEvent& event)
{
    if (event.button != 0 || !mPressed) return;
    const bool grabbed = mGrabbing;
    mPressed = false;
    mGrabbing = false;
    event.consumed = true;
    if (!mDragged && !grabbed) beginEdit();
    mDragged = false;
    mOverGrabber = overGrabber(event.localX, event.localY);
    markDirty();
}

void NumberField::onMouseMove(rt::MouseEvent& event)
{
    if (!mPressed)
    {
        const bool over = overGrabber(event.localX, event.localY);
        if (over != mOverGrabber)
        {
            mOverGrabber = over;
            markDirty();
        }
        return;
    }
    if (mGrabbing)
    {
        mDragged = true;
        event.consumed = true;
        setRatio(mGrabRatio + (event.x - mGrabFrom) / trackWidth());
        return;
    }
    const float delta = event.x - mPressX;
    if (!mDragged)
    {
        if (fabsf(delta) <= kDragThreshold) return;
        mDragged = true;
    }
    event.consumed = true;
    float scale = mSensitivity;
    if (event.shift) scale *= kShiftScale;
    double value = static_cast<double>(mPressValue) + static_cast<double>(delta) * scale;
    if (event.ctrl) value = floor(value + 0.5);
    if (mRanged)
    {
        if (!mOrLess && value < mMin) value = mMin;
        if (!mOrGreater && value > mMax) value = mMax;
    }
    setValueInternal(static_cast<float>(value), true);
}

void NumberField::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape && (mPressed || mGrabbing))
    {
        cancelDrag();
        event.consumed = true;
        return;
    }
    rt::Widget::onKeyPress(event);
}

} // namespace prisma3d
