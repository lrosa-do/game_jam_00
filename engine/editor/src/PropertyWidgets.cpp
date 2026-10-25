#include "prisma3d/editor/PropertyWidgets.h"

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kLabelHeight = 26.0f;
const float kSwatchWidth = 72.0f;
const float kSwatchHeight = 26.0f;
const float kPopupWidth = 230.0f;
const float kPopupHeight = 270.0f;
const float kPopupPadding = 6.0f;
const float kLineHeightRatio = 1.2f;
const float kArrowSize = 14.0f;
const std::int64_t kAllBits = 0xffffffffll;

rt::Color toColor(const Math::Vec4& value)
{
    const auto channel = [](float v)
    {
        const float clamped = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
        return static_cast<unsigned char>(roundf(clamped * 255.0f));
    };
    return rt::Color(channel(value.x), channel(value.y), channel(value.z), channel(value.w));
}

} // namespace

NameLabel::NameLabel(const ct::String& text)
        : mText(text)
{
}

const ct::String& NameLabel::text() const
{
    return mText;
}

void NameLabel::setText(const ct::String& text)
{
    mText = text;
    markDirty();
}

rt::Widget::Vec2f NameLabel::sizeHint() const
{
    return { 0.0f, kLabelHeight };
}

void NameLabel::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    if (overrideFont_) ctx.pushFont(overrideFont_);
    ctx.pushClip(abs);
    ctx.font.SetFontSize(theme.fontSize);
    ctx.font.SetBatch(&ctx.text);
    const float ascender = ctx.font.GetAscender();
    ctx.font.SetColor(enabled_ ? theme.textColor : theme.textDisabled);
    const float room = abs.w - 4.0f;
    ct::String shown = mText;
    if (ctx.font.GetTextWidth(shown.c_str()) > room)
    {
        const char* const ellipsis = "\xE2\x80\xA6";
        while (!shown.empty())
        {
            shown.pop_back();
            while (!shown.empty() && (static_cast<unsigned char>(shown.back()) & 0xC0) == 0x80)
                shown.pop_back();
            if (ctx.font.GetTextWidth((shown + ellipsis).c_str()) <= room) break;
        }
        shown += ellipsis;
    }
    ctx.font.Print(shown.c_str(), abs.x + 2.0f, abs.y + (abs.h + ascender) * 0.5f);
    ctx.popClip();
    if (overrideFont_) ctx.popFont();
}

CategoryHeader::CategoryHeader(const ct::String& text, rt::IconId icon)
        : mText(text),
          mIcon(icon)
{
}

rt::IconId CategoryHeader::icon() const
{
    return mIcon;
}

const ct::String& CategoryHeader::text() const
{
    return mText;
}

rt::Widget::Vec2f CategoryHeader::sizeHint() const
{
    return { 0.0f, kLabelHeight };
}

void CategoryHeader::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    ctx.fill.SetColor(theme.buttonNormal);
    ctx.fill.Rectangle(abs.x, abs.y, abs.w, abs.h, true);

    if (overrideFont_) ctx.pushFont(overrideFont_);
    ctx.pushClip(abs);
    ctx.font.SetFontSize(theme.fontSize);
    ctx.font.SetBatch(&ctx.text);
    const float ascender = ctx.font.GetAscender();
    const float width = ctx.font.GetTextWidth(mText.c_str());
    const bool hasIcon = mIcon != rt::IconId::None;
    const float lead = hasIcon ? kIconSize + kIconGap : 0.0f;
    float left = abs.x + (abs.w - width - lead) * 0.5f;
    if (left < abs.x + 4.0f) left = abs.x + 4.0f;
    if (hasIcon)
    {
        ctx.drawIcon(mIcon, floorf(left), floorf(abs.y + (abs.h - kIconSize) * 0.5f), kIconSize);
        left += lead;
    }
    ctx.font.SetColor(theme.textColor);
    ctx.font.Print(mText.c_str(), left, abs.y + (abs.h + ascender) * 0.5f);
    ctx.popClip();
    if (overrideFont_) ctx.popFont();
}

RevertButton::RevertButton()
        : mActive(false)
{
    setTooltip("Revert to the default value");
    cursor_ = rt::CursorType::Hand;
}

bool RevertButton::active() const
{
    return mActive;
}

void RevertButton::setActive(bool active)
{
    if (mActive == active) return;
    mActive = active;
    markDirty();
}

rt::Widget::Vec2f RevertButton::sizeHint() const
{
    return { kSize, kLabelHeight };
}

void RevertButton::paint(rt::PaintContext& ctx)
{
    if (!visible_ || !mActive) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    rt::Rect clipped;
    if (isHovered() && ctx.clipRectIntersect(abs, clipped))
    {
        ctx.fill.SetColor(theme.buttonHover);
        ctx.fill.RoundedRectangle(clipped.x, clipped.y, clipped.w, clipped.h, theme.borderRadius, 6,
                true);
    }
    const rt::Color color = theme.textColor;
    const rt::Vec2 center = { abs.x + abs.w * 0.5f, abs.y + abs.h * 0.5f };
    const float radius = 4.8f;
    ctx.drawList.pathClear();
    ctx.drawList.pathArcToFast(center, radius, 0, 9);
    ctx.drawList.pathStroke(color, 1.6f, false);
    const rt::Vec2 tip = { center.x + 3.6f, center.y - radius };
    const rt::Vec2 upper = { center.x - 1.2f, center.y - radius - 3.4f };
    const rt::Vec2 lower = { center.x - 1.2f, center.y - radius + 3.4f };
    ctx.drawList.addTriangleFilled(tip, upper, lower, color);
}

void RevertButton::onMouseRelease(rt::MouseEvent& event)
{
    const bool wasPressed = pressed_;
    rt::Widget::onMouseRelease(event);
    if (!mActive || !wasPressed || event.button != 0) return;
    if (!absoluteRect().contains(event.x, event.y)) return;
    reverted.emit();
}

CommitInput::CommitInput() = default;

const ct::String& CommitInput::committedText() const
{
    return mCommitted;
}

void CommitInput::setCommittedText(const ct::String& text)
{
    mCommitted = text;
    setText(text);
}

bool CommitInput::pending() const
{
    return text() != mCommitted;
}

void CommitInput::commit()
{
    if (!pending()) return;
    mCommitted = text();
    committed.emit(mCommitted);
}

void CommitInput::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape)
    {
        setText(mCommitted);
        event.consumed = true;
        return;
    }
    const bool submit = event.key == rt::Key::Return || event.key == rt::Key::KPEnter;
    if (submit) commit();
    rt::TextInput::onKeyPress(event);
    if (!submit) return;
    if (rt::WidgetApp* app = rt::WidgetApp::existing())
        if (app->focused() == this) app->setFocused(nullptr);
}

void CommitInput::onFocusLost()
{
    commit();
}

LayerGrid::LayerGrid(int count, int groupSize)
        : mBits(0),
          mCount(count < 1 ? 1 : (count > kMaxLayers ? kMaxLayers : count)),
          mGroupSize(groupSize < 1 ? 1 : groupSize),
          mHovered(-1),
          mWidths{},
          mHintedRows(0),
          mExpanded(false),
          mExpandHovered(false),
          mDragging(false),
          mDraggingValue(false)
{
    cursor_ = rt::CursorType::Hand;
}

int LayerGrid::count() const
{
    return mCount;
}

int LayerGrid::groupSize() const
{
    return mGroupSize;
}

void LayerGrid::computeLayout(Layout& layout, float width) const
{
    const rt::Theme& theme = rt::Theme::instance();
    layout.gridHeight = theme.fontSize * kLineHeightRatio * 3.0f;
    const int blockSize = static_cast<int>(layout.gridHeight * 80.0f / 100.0f / 2.0f);
    const int height = blockSize * 2 + 1;
    const int verticalOffset =
            static_cast<int>((layout.gridHeight - static_cast<float>(height)) / 2.0f);
    layout.rowHeight = static_cast<float>(2 * (blockSize + 1) + 3);
    layout.visible = 0;
    layout.expansionRows = 0;
    layout.arrowShown = false;
    layout.arrow = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < kMaxLayers; ++i) layout.cells[i] = { 0.0f, 0.0f, 0.0f, 0.0f };

    const float bsize = static_cast<float>(blockSize);
    float blockX = 4.0f;
    float blockY = static_cast<float>(verticalOffset);
    int layerIndex = 0;
    float arrowX = 0.0f;
    float arrowY = 0.0f;
    bool arrowSet = false;

    while (true)
    {
        float x = blockX;
        float y = blockY;
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < mGroupSize; ++j)
            {
                if (layerIndex < mCount)
                {
                    layout.cells[layerIndex] = { x, y, bsize, bsize };
                    layout.visible = layerIndex + 1;
                    ++layerIndex;
                }
                x += bsize + 1.0f;
            }
            x = blockX;
            y += bsize + 1.0f;
        }

        if (layerIndex >= mCount)
        {
            if (layout.visible > 0 && layout.expansionRows == 0 && !arrowSet)
            {
                const rt::Rect& last = layout.cells[layout.visible - 1];
                arrowX = last.x + last.w;
                arrowY = last.y + last.h;
                arrowSet = true;
            }
            break;
        }

        const float blockWidth = static_cast<float>(mGroupSize) * (bsize + 1.0f);
        blockX += blockWidth + 3.0f;

        if (blockX + blockWidth + 12.0f > width)
        {
            if (layout.visible > 0 && layout.expansionRows == 0 && !arrowSet)
            {
                const rt::Rect& last = layout.cells[layout.visible - 1];
                arrowX = last.x + last.w;
                arrowY = last.y + last.h;
                arrowSet = true;
            }
            ++layout.expansionRows;
            if (mExpanded)
            {
                blockX = 4.0f;
                blockY += layout.rowHeight;
            }
            else
                break;
        }
    }

    if (layout.expansionRows == 0 && layerIndex == mCount) return;
    layout.arrowShown = true;
    layout.arrow = { arrowX + 2.0f, arrowY - kArrowSize, kArrowSize, kArrowSize };
}

int LayerGrid::visibleCount() const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    return layout.visible;
}

int LayerGrid::expansionRows() const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    return layout.expansionRows;
}

bool LayerGrid::expanded() const
{
    return mExpanded;
}

void LayerGrid::setExpanded(bool expanded)
{
    if (mExpanded == expanded) return;
    mExpanded = expanded;
    markDirty();
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

bool LayerGrid::expandable() const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    return layout.arrowShown;
}

bool LayerGrid::isChecked(int index) const
{
    return index >= 0 && index < mCount && ((mBits >> index) & 1) != 0;
}

void LayerGrid::setChecked(int index, bool checked)
{
    if (index < 0 || index >= mCount) return;
    const std::int64_t bit = static_cast<std::int64_t>(1) << index;
    if (checked) mBits |= bit;
    else
        mBits &= ~bit;
    markDirty();
}

std::int64_t LayerGrid::bits() const
{
    return mBits;
}

void LayerGrid::setBits(std::int64_t bits)
{
    mBits = bits;
    markDirty();
}

rt::Rect LayerGrid::cellRect(int index) const
{
    if (index < 0 || index >= mCount) return { 0.0f, 0.0f, 0.0f, 0.0f };
    Layout layout;
    computeLayout(layout, rect_.w);
    const rt::Rect abs = absoluteRect();
    const rt::Rect& cell = layout.cells[index];
    return { abs.x + cell.x, abs.y + cell.y, cell.w, cell.h };
}

rt::Rect LayerGrid::expandRect() const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    const rt::Rect abs = absoluteRect();
    return { abs.x + layout.arrow.x, abs.y + layout.arrow.y, layout.arrow.w, layout.arrow.h };
}

int LayerGrid::cellAt(float x, float y) const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    const rt::Rect abs = absoluteRect();
    for (int i = 0; i < layout.visible; ++i)
    {
        const rt::Rect& cell = layout.cells[i];
        const rt::Rect rect = { abs.x + cell.x, abs.y + cell.y, cell.w, cell.h };
        if (rect.w > 0.0f && rect.contains(x, y)) return i;
    }
    return -1;
}

bool LayerGrid::overExpand(float x, float y) const
{
    Layout layout;
    computeLayout(layout, rect_.w);
    if (!layout.arrowShown) return false;
    const rt::Rect abs = absoluteRect();
    const rt::Rect rect = { abs.x + layout.arrow.x, abs.y + layout.arrow.y, layout.arrow.w,
        layout.arrow.h };
    return rect.contains(x, y);
}

int LayerGrid::hovered() const
{
    return mHovered;
}

bool LayerGrid::expandHovered() const
{
    return mExpandHovered;
}

ct::String LayerGrid::layerTooltip(int index)
{
    ct::String text = "Layer ";
    text.append_number(index + 1);
    text += "\nBit ";
    text.append_number(index);
    text += ", value ";
    text.append_number(static_cast<unsigned long long>(1) << index);
    return text;
}

void LayerGrid::layout()
{
    if (rect_.w == mWidths[0]) return;
    mWidths[1] = mWidths[0];
    mWidths[0] = rect_.w;
}

float LayerGrid::hintWidth() const
{
    return mWidths[1] > 0.0f && mWidths[1] < mWidths[0] ? mWidths[1] : mWidths[0];
}

ct::String LayerGrid::cellLabel(int index)
{
    ct::String text;
    text.append_number(index + 1);
    return text;
}

float LayerGrid::labelX(const rt::Rect& cell, float textWidth)
{
    return cell.x + (cell.w - textWidth) * 0.5f;
}

float LayerGrid::labelBaseline(const rt::Rect& cell)
{
    return cell.y + cell.h * 0.75f;
}

float LayerGrid::cellAlpha(bool on, bool hovered)
{
    return (on ? 0.6f : 0.2f) + (hovered ? 0.15f : 0.0f);
}

float LayerGrid::textAlpha(bool on)
{
    return on ? 0.7f : 0.5f;
}

rt::Widget::Vec2f LayerGrid::sizeHint() const
{
    Layout layout;
    computeLayout(layout, hintWidth());
    float height = layout.gridHeight;
    if (mExpanded) height += static_cast<float>(layout.expansionRows) * layout.rowHeight;
    return { 0.0f, height };
}

void LayerGrid::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    Layout layout;
    computeLayout(layout, rect_.w);
    if (layout.expansionRows != mHintedRows)
    {
        mHintedRows = layout.expansionRows;
        if (mExpanded)
            if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
    }

    const auto channel = [](float alpha) { return static_cast<unsigned char>(alpha * 255.0f); };
    ctx.font.SetFontSize(theme.fontSize);
    ctx.font.SetBatch(&ctx.text);
    for (int i = 0; i < layout.visible; ++i)
    {
        const rt::Rect cell = { abs.x + layout.cells[i].x, abs.y + layout.cells[i].y,
            layout.cells[i].w, layout.cells[i].h };
        const bool on = isChecked(i);
        ctx.fill.SetColor(theme.focusColor.r, theme.focusColor.g, theme.focusColor.b,
                channel(cellAlpha(on, i == mHovered)));
        ctx.fillRect(cell.x, cell.y, cell.w, cell.h);

        const ct::String number = cellLabel(i);
        const rt::Color base = on ? theme.menuItemTextHover : theme.textColor;
        ctx.font.SetColor(rt::Color(base.r, base.g, base.b, channel(textAlpha(on))));
        const float width = ctx.font.GetTextWidth(number.c_str());
        ctx.pushClip(cell);
        ctx.font.Print(number.c_str(), labelX(cell, width), labelBaseline(cell));
        ctx.popClip();
    }

    if (layout.arrowShown)
    {
        const rt::Color accent = theme.focusColor;
        const rt::Color color(accent.r, accent.g, accent.b, channel(mExpandHovered ? 1.0f : 0.6f));
        const float x = abs.x + layout.arrow.x;
        const float y = abs.y + layout.arrow.y;
        const float size = layout.arrow.w;
        const float inset = size * 0.2f;
        if (mExpanded)
            ctx.drawList.addTriangleFilled({ x + inset, y + size - inset },
                    { x + size - inset, y + size - inset }, { x + size * 0.5f, y + inset }, color);
        else
            ctx.drawList.addTriangleFilled({ x + inset, y + inset },
                    { x + size - inset, y + inset }, { x + size * 0.5f, y + size - inset }, color);
    }
}

void LayerGrid::setHovered(int index, bool expandHovered)
{
    if (index == mHovered && expandHovered == mExpandHovered) return;
    mHovered = index;
    mExpandHovered = expandHovered;
    if (index >= 0) setTooltip(layerTooltip(index));
    markDirty();
}

void LayerGrid::updateHovered(float x, float y)
{
    if (overExpand(x, y))
    {
        setHovered(-1, true);
        return;
    }
    setHovered(cellAt(x, y), false);
}

void LayerGrid::updateFlag(bool replace)
{
    if (mHovered >= 0)
    {
        const std::int64_t bit = static_cast<std::int64_t>(1) << mHovered;
        if (replace)
        {
            if (mBits == bit) mBits = ~mBits & kAllBits;
            else
                mBits = bit;
        }
        else
            mBits ^= bit;
        markDirty();
        changed.emit(mBits);
    }
    else if (mExpandHovered)
    {
        setExpanded(!mExpanded);
    }
}

void LayerGrid::onMousePress(rt::MouseEvent& event)
{
    if (!enabled_ || event.button != 0) return;
    updateHovered(event.x, event.y);
    const bool replace = event.ctrl;
    updateFlag(replace);
    if (!replace && mHovered >= 0)
    {
        mDragging = true;
        mDraggingValue = isChecked(mHovered);
    }
    event.consumed = true;
}

void LayerGrid::onMouseRelease(rt::MouseEvent& event)
{
    if (event.button != 0) return;
    mDragging = false;
    event.consumed = true;
}

void LayerGrid::onMouseLeave()
{
    mDragging = false;
    if (mHovered < 0 && !mExpandHovered) return;
    mHovered = -1;
    mExpandHovered = false;
    markDirty();
}

void LayerGrid::onMouseMove(rt::MouseEvent& event)
{
    if (!enabled_) return;
    updateHovered(event.x, event.y);
    if (mDragging && mHovered >= 0 && mDraggingValue != isChecked(mHovered))
    {
        mBits ^= static_cast<std::int64_t>(1) << mHovered;
        markDirty();
        changed.emit(mBits);
    }
}

class ColorPopup : public rt::Widget
{
public:
    IG_WIDGET_CLASS(ColorPopup, rt::Widget);

    ColorPopup(const ct::Rc<ColorSwatch::Link>& link, bool alpha, const rt::Color& color)
            : mLink(link),
              mPicker(nullptr)
    {
        acceptsFocus_ = true;
        mPicker = createChild<rt::ColorPicker>();
        mPicker->setShowAlpha(alpha);
        mPicker->setColor(color);
        mPicker->colorChanged.connect(
                [this](const rt::Color& changed)
                {
                    if (mLink->owner) mLink->owner->pickerChanged(changed);
                });
    }

    ~ColorPopup() override
    {
        notifyClosed();
    }

    rt::ColorPicker* picker() const
    {
        return mPicker;
    }

    void fit()
    {
        mPicker->setRect({ kPopupPadding, kPopupPadding, rect_.w - kPopupPadding * 2.0f,
            rect_.h - kPopupPadding * 2.0f });
    }

    void resetPopupState() override
    {
        notifyClosed();
    }

    void layout() override
    {
        fit();
    }

    void paint(rt::PaintContext& ctx) override
    {
        const rt::Rect abs = absoluteRect();
        const rt::Theme& theme = rt::Theme::instance();
        ctx.fill.SetColor(0, 0, 0, 60);
        ctx.fillRect(abs.x + 3.0f, abs.y + 3.0f, abs.w, abs.h);
        ctx.fill.SetColor(theme.panelColor.r, theme.panelColor.g, theme.panelColor.b, 250);
        ctx.fillRoundedRect(abs.x, abs.y, abs.w, abs.h, theme.borderRadius, 6);
        ctx.line.SetColor(theme.inputBorderHover);
        ctx.lineRoundedRect(abs.x, abs.y, abs.w, abs.h, theme.borderRadius, 6);
        fit();
        rt::Widget::paint(ctx);
    }

private:
    void notifyClosed()
    {
        if (mLink->owner) mLink->owner->popupClosed(this);
    }

    ct::Rc<ColorSwatch::Link> mLink;
    rt::ColorPicker* mPicker;
};

ColorSwatch::ColorSwatch()
        : mLink(ct::make_rc<Link>(Link{ this })),
          mColor(1.0f, 1.0f, 1.0f, 1.0f),
          mPopup(nullptr),
          mAlpha(true)
{
    cursor_ = rt::CursorType::Hand;
    clicked.connect(
            [this]
            {
                if (isOpen()) close();
                else
                    open();
            });
}

ColorSwatch::~ColorSwatch()
{
    mLink->owner = nullptr;
}

const Math::Vec4& ColorSwatch::color() const
{
    return mColor;
}

void ColorSwatch::setColor(const Math::Vec4& color)
{
    mColor = color;
    markDirty();
}

bool ColorSwatch::alphaEnabled() const
{
    return mAlpha;
}

void ColorSwatch::setAlphaEnabled(bool enabled)
{
    mAlpha = enabled;
}

bool ColorSwatch::isOpen() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return mPopup && app && app->popup() == mPopup;
}

void ColorSwatch::open()
{
    if (isOpen() || !enabled_) return;
    const rt::Rect abs = absoluteRect();
    ColorPopup* popup = new ColorPopup(mLink, mAlpha, toColor(mColor));
    const rt::IO& io = rt::GetIO();
    float x = abs.x;
    float y = abs.y + abs.h + 2.0f;
    if (x + kPopupWidth > io.displayWidth) x = io.displayWidth - kPopupWidth;
    if (y + kPopupHeight > io.displayHeight)
        y = abs.y - kPopupHeight >= 0.0f ? abs.y - kPopupHeight : io.displayHeight - kPopupHeight;
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    popup->setRect({ x, y, kPopupWidth, kPopupHeight });
    popup->fit();
    mPopup = popup;
    rt::WidgetApp::instance().showPopup(popup, this, true);
}

void ColorSwatch::close()
{
    if (!isOpen()) return;
    rt::WidgetApp::instance().closePopup();
}

rt::ColorPicker* ColorSwatch::picker() const
{
    return isOpen() ? mPopup->picker() : nullptr;
}

rt::Widget::Vec2f ColorSwatch::sizeHint() const
{
    return { kSwatchWidth, kSwatchHeight };
}

void ColorSwatch::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    const float half = abs.w * 0.5f;
    ctx.fill.SetColor(60, 60, 60, 255);
    ctx.fillRect(abs.x, abs.y, half, abs.h);
    ctx.fill.SetColor(110, 110, 110, 255);
    ctx.fillRect(abs.x + half, abs.y, abs.w - half, abs.h);

    const rt::Color shown = toColor(mColor);
    ctx.fill.SetColor(shown.r, shown.g, shown.b, shown.a);
    ctx.fillRect(abs.x, abs.y, abs.w, abs.h);

    ctx.line.SetColor(isHovered() || isOpen() ? theme.inputBorderHover : theme.inputBorder);
    ctx.lineRoundedRect(abs.x, abs.y, abs.w, abs.h, theme.borderRadius, 6);
}

void ColorSwatch::pickerChanged(const rt::Color& color)
{
    Math::Vec4 changed(static_cast<float>(color.r) / 255.0f, static_cast<float>(color.g) / 255.0f,
            static_cast<float>(color.b) / 255.0f, static_cast<float>(color.a) / 255.0f);
    if (!mAlpha) changed.w = mColor.w;
    mColor = changed;
    markDirty();
    colorEdited.emit(mColor);
}

void ColorSwatch::popupClosed(ColorPopup* popup)
{
    if (mPopup == popup) mPopup = nullptr;
}

} // namespace prisma3d
