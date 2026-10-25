#include "prisma3d/editor/AssetGrid.h"

#include <igui/widgets/Theme.hpp>

#include <any>
#include <math.h>
#include <stdio.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kPad = 8.0f;
const float kInset = 8.0f;
const float kLabelHeight = 22.0f;
const float kBarWidth = 10.0f;
const float kBarMinThumb = 24.0f;
const float kTextInset = 6.0f;
const float kIconSize = 16.0f;

ct::String formatSize(std::uint64_t bytes)
{
    char buffer[48];
    if (bytes < 1024)
        snprintf(buffer, sizeof buffer, "%llu B", static_cast<unsigned long long>(bytes));
    else if (bytes < 1024ull * 1024ull)
        snprintf(buffer, sizeof buffer, "%.1f KB", static_cast<double>(bytes) / 1024.0);
    else if (bytes < 1024ull * 1024ull * 1024ull)
        snprintf(buffer, sizeof buffer, "%.1f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    else
        snprintf(buffer, sizeof buffer, "%.2f GB",
                static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0));
    return ct::String(buffer);
}

ct::String upper(const ct::String& text)
{
    ct::String result = text;
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        const char c = result[i];
        if (c >= 'a' && c <= 'z') result.data()[i] = static_cast<char>(c - 'a' + 'A');
    }
    return result;
}

ct::String fitted(rt::PaintContext& ctx, const ct::String& text, float width)
{
    if (ctx.font.GetTextWidth(text.c_str()) <= width) return text;
    ct::String result = text;
    while (!result.empty())
    {
        result.pop_back();
        while (!result.empty() && (static_cast<unsigned char>(result.back()) & 0xC0) == 0x80)
            result.pop_back();
        const ct::String candidate = result + "...";
        if (ctx.font.GetTextWidth(candidate.c_str()) <= width) return candidate;
    }
    return ct::String("...");
}

void print(rt::PaintContext& ctx, const ct::String& text, float x, float top, float height,
        const rt::Color& color)
{
    if (text.empty()) return;
    const float ascender = ctx.font.GetAscender();
    ctx.font.SetColor(color);
    ctx.font.Print(text.c_str(), x, top + (height + ascender) * 0.5f);
}

} // namespace

AssetGrid::AssetGrid()
        : mMode(AssetViewMode::Grid),
          mThumbnail(96.0f),
          mScroll(0.0f),
          mSelected(-1),
          mHovered(-1),
          mPressed(-1),
          mDragged(-1),
          mActivate(-1),
          mBarDrag(false),
          mBarGrab(0.0f)
{
}

void AssetGrid::setItems(const ct::Vector<AssetEntry>& items)
{
    mItems = items;
    mSelected = -1;
    mHovered = -1;
    mPressed = -1;
    setScroll(mScroll);
    markDirty();
}

std::size_t AssetGrid::itemCount() const
{
    return mItems.size();
}

const AssetEntry& AssetGrid::item(std::size_t index) const
{
    return mItems[index];
}

void AssetGrid::setViewMode(AssetViewMode mode)
{
    if (mode == mMode) return;
    mMode = mode;
    setScroll(mScroll);
    markDirty();
}

AssetViewMode AssetGrid::viewMode() const
{
    return mMode;
}

void AssetGrid::setThumbnailSize(float size)
{
    if (size < kMinThumbnail) size = kMinThumbnail;
    if (size > kMaxThumbnail) size = kMaxThumbnail;
    if (size == mThumbnail) return;
    mThumbnail = size;
    setScroll(mScroll);
    markDirty();
}

float AssetGrid::thumbnailSize() const
{
    return mThumbnail;
}

void AssetGrid::setThumbnailProvider(const ThumbnailProvider& provider)
{
    mProvider = provider;
}

int AssetGrid::selected() const
{
    return mSelected;
}

void AssetGrid::setSelected(int index)
{
    if (index < 0 || index >= static_cast<int>(mItems.size())) index = -1;
    if (index == mSelected) return;
    mSelected = index;
    if (index >= 0) ensureVisible(index);
    markDirty();
}

int AssetGrid::hovered() const
{
    return mHovered;
}

float AssetGrid::top() const
{
    return mMode == AssetViewMode::Details ? kHeaderHeight : 0.0f;
}

float AssetGrid::viewportHeight() const
{
    const float height = rect_.h - top();
    return height > 0.0f ? height : 0.0f;
}

float AssetGrid::cellWidth() const
{
    return mThumbnail + kInset * 2.0f;
}

float AssetGrid::cellHeight() const
{
    if (mMode == AssetViewMode::Grid) return mThumbnail + kLabelHeight + kInset;
    return kRowHeight;
}

int AssetGrid::columns() const
{
    if (mMode != AssetViewMode::Grid) return 1;
    const int count = static_cast<int>((rect_.w - kPad * 2.0f - kBarWidth) / cellWidth());
    return count < 1 ? 1 : count;
}

float AssetGrid::contentHeight() const
{
    const int cols = columns();
    const int rows = (static_cast<int>(mItems.size()) + cols - 1) / cols;
    const float padding = mMode == AssetViewMode::Grid ? kPad * 2.0f : 0.0f;
    return static_cast<float>(rows) * cellHeight() + padding;
}

float AssetGrid::maxScroll() const
{
    const float extra = contentHeight() - viewportHeight();
    return extra > 0.0f ? extra : 0.0f;
}

float AssetGrid::scroll() const
{
    return mScroll;
}

void AssetGrid::setScroll(float value)
{
    const float limit = maxScroll();
    if (value > limit) value = limit;
    if (value < 0.0f) value = 0.0f;
    if (value == mScroll) return;
    mScroll = value;
    markDirty();
}

void AssetGrid::ensureVisible(int index)
{
    if (index < 0 || index >= static_cast<int>(mItems.size())) return;
    const int cols = columns();
    const float pad = mMode == AssetViewMode::Grid ? kPad : 0.0f;
    const float itemTop = pad + static_cast<float>(index / cols) * cellHeight();
    const float itemBottom = itemTop + cellHeight();
    if (itemTop < mScroll) setScroll(itemTop - pad);
    else if (itemBottom > mScroll + viewportHeight())
        setScroll(itemBottom - viewportHeight() + pad);
}

rt::Rect AssetGrid::itemRect(int index) const
{
    const rt::Rect abs = absoluteRect();
    const int cols = columns();
    const int column = index % cols;
    const int row = index / cols;
    if (mMode == AssetViewMode::Grid)
        return rt::Rect{ abs.x + kPad + static_cast<float>(column) * cellWidth(),
            abs.y + top() + kPad + static_cast<float>(row) * cellHeight() - mScroll, cellWidth(),
            cellHeight() };
    return rt::Rect{ abs.x, abs.y + top() + static_cast<float>(row) * cellHeight() - mScroll,
        abs.w - kBarWidth, cellHeight() };
}

int AssetGrid::indexAt(float x, float y) const
{
    const rt::Rect abs = absoluteRect();
    if (!abs.contains(x, y) || y < abs.y + top() || x >= abs.x + abs.w - kBarWidth) return -1;
    const int cols = columns();
    float relativeX = x - abs.x;
    float relativeY = y - abs.y - top() + mScroll;
    int column = 0;
    if (mMode == AssetViewMode::Grid)
    {
        relativeX -= kPad;
        relativeY -= kPad;
        if (relativeX < 0.0f || relativeY < 0.0f) return -1;
        column = static_cast<int>(relativeX / cellWidth());
        if (column >= cols) return -1;
    }
    else if (relativeY < 0.0f)
        return -1;
    const int row = static_cast<int>(relativeY / cellHeight());
    const int index = row * cols + column;
    return index < static_cast<int>(mItems.size()) ? index : -1;
}

rt::Rect AssetGrid::barRect() const
{
    const rt::Rect abs = absoluteRect();
    return rt::Rect{ abs.x + abs.w - kBarWidth, abs.y + top(), kBarWidth, viewportHeight() };
}

rt::Rect AssetGrid::thumbRect() const
{
    const rt::Rect bar = barRect();
    const float total = contentHeight();
    if (total <= 0.0f || maxScroll() <= 0.0f) return rt::Rect{ bar.x, bar.y, bar.w, 0.0f };
    float length = bar.h * (viewportHeight() / total);
    if (length < kBarMinThumb) length = kBarMinThumb;
    if (length > bar.h) length = bar.h;
    const float travel = bar.h - length;
    const float offset = travel * (mScroll / maxScroll());
    return rt::Rect{ bar.x, bar.y + offset, bar.w, length };
}

bool AssetGrid::overBar(float x, float y) const
{
    return maxScroll() > 0.0f && barRect().contains(x, y);
}

void AssetGrid::dragBar(float y)
{
    const rt::Rect bar = barRect();
    const rt::Rect thumb = thumbRect();
    const float travel = bar.h - thumb.h;
    if (travel <= 0.0f) return;
    const float offset = y - mBarGrab - bar.y;
    setScroll(maxScroll() * (offset / travel));
}

rt::Widget::Vec2f AssetGrid::sizeHint() const
{
    return { 240.0f, 160.0f };
}

void AssetGrid::paintHeader(rt::PaintContext& ctx)
{
    const rt::Theme& theme = rt::Theme::instance();
    const rt::Rect abs = absoluteRect();
    ctx.fill.SetColor(theme.panelColor);
    ctx.fill.Rectangle(abs.x, abs.y, abs.w, kHeaderHeight, true);
    const float typeX = abs.x + abs.w * 0.55f;
    const float sizeX = abs.x + abs.w - kBarWidth - 72.0f;
    print(ctx, ct::String("Name"), abs.x + kTextInset + kIconSize + kTextInset, abs.y,
            kHeaderHeight, theme.textDisabled);
    print(ctx, ct::String("Type"), typeX, abs.y, kHeaderHeight, theme.textDisabled);
    print(ctx, ct::String("Size"), sizeX, abs.y, kHeaderHeight, theme.textDisabled);
    ctx.line.SetColor(theme.borderColor);
    ctx.line.Line2D(abs.x, abs.y + kHeaderHeight, abs.x + abs.w, abs.y + kHeaderHeight);
}

void AssetGrid::paintGridItem(rt::PaintContext& ctx, const AssetEntry& entry, const rt::Rect& cell)
{
    const rt::Theme& theme = rt::Theme::instance();
    const float size = mThumbnail;
    const float thumbX = cell.x + kInset;
    const float thumbY = cell.y + kInset * 0.5f;

    const AssetThumbnail thumbnail = mProvider ? mProvider(entry) : AssetThumbnail();
    if (thumbnail.texture)
    {
        float width = size;
        float height = size;
        if (thumbnail.aspect > 1.0f) height = size / thumbnail.aspect;
        else if (thumbnail.aspect > 0.0f)
            width = size * thumbnail.aspect;
        const rt::Rect target{ thumbX + (size - width) * 0.5f, thumbY + (size - height) * 0.5f,
            width, height };
        ctx.drawImage(thumbnail.texture, target);
        ctx.line.SetColor(theme.borderColor);
        ctx.line.Rectangle(target.x, target.y, target.w, target.h, false);
    }
    else
    {
        const AssetIcon icon = AssetTypes::icon(entry.kind);
        const float iconSize = size * 0.6f;
        ctx.drawIcon(icon.icon, thumbX + (size - iconSize) * 0.5f,
                thumbY + (size - iconSize) * 0.5f, iconSize, icon.tint);
    }

    if (!entry.directory)
    {
        const ct::String extension = upper(AssetTypes::extensionOf(entry.name));
        if (!extension.empty())
        {
            const float textWidth = ctx.font.GetTextWidth(extension.c_str());
            const float badgeWidth = textWidth + 8.0f;
            const float badgeHeight = ctx.font.GetAscender() + 6.0f;
            const rt::Rect badge{ thumbX + size - badgeWidth, thumbY + size - badgeHeight,
                badgeWidth, badgeHeight };
            ctx.fill.SetColor(rt::Color(20, 24, 30, 210));
            ctx.fill.RoundedRectangle(badge.x, badge.y, badge.w, badge.h, 3.0f, 4, true);
            print(ctx, extension, badge.x + 4.0f, badge.y, badge.h, rt::Color(235, 238, 245, 255));
        }
    }

    const float labelWidth = cell.w - 8.0f;
    const ct::String label = fitted(ctx, entry.name, labelWidth);
    const float textWidth = ctx.font.GetTextWidth(label.c_str());
    print(ctx, label, cell.x + (cell.w - textWidth) * 0.5f, thumbY + size, kLabelHeight,
            theme.textColor);
}

void AssetGrid::paintRowItem(rt::PaintContext& ctx, const AssetEntry& entry, const rt::Rect& cell)
{
    const rt::Theme& theme = rt::Theme::instance();
    const AssetIcon icon = AssetTypes::icon(entry.kind);
    ctx.drawIcon(icon.icon, cell.x + kTextInset, cell.y + (cell.h - kIconSize) * 0.5f, kIconSize,
            icon.tint);
    const float nameX = cell.x + kTextInset + kIconSize + kTextInset;
    float nameWidth = cell.w - (nameX - cell.x) - kTextInset;
    if (mMode == AssetViewMode::Details)
    {
        const float typeX = cell.x + cell.w * 0.55f;
        const float sizeX = cell.x + cell.w - 72.0f;
        nameWidth = typeX - nameX - kTextInset;
        print(ctx, ct::String(AssetTypes::label(entry.kind)), typeX, cell.y, cell.h,
                theme.textDisabled);
        if (!entry.directory)
            print(ctx, formatSize(entry.size), sizeX, cell.y, cell.h, theme.textDisabled);
    }
    print(ctx, fitted(ctx, entry.name, nameWidth), nameX, cell.y, cell.h, theme.textColor);
}

void AssetGrid::paintItem(rt::PaintContext& ctx, int index, const rt::Rect& cell)
{
    const rt::Theme& theme = rt::Theme::instance();
    const bool selected = index == mSelected;
    const bool hovered = index == mHovered;
    if (selected || hovered)
    {
        ctx.fill.SetColor(selected ? theme.selectionColor : theme.menuItemHover);
        if (mMode == AssetViewMode::Grid)
            ctx.fill.RoundedRectangle(cell.x, cell.y, cell.w, cell.h, theme.borderRadius, 6, true);
        else
            ctx.fill.Rectangle(cell.x, cell.y, cell.w, cell.h, true);
    }
    const AssetEntry& entry = mItems[static_cast<std::size_t>(index)];
    if (mMode == AssetViewMode::Grid) paintGridItem(ctx, entry, cell);
    else
        paintRowItem(ctx, entry, cell);
}

void AssetGrid::paintBar(rt::PaintContext& ctx)
{
    if (maxScroll() <= 0.0f) return;
    const rt::Theme& theme = rt::Theme::instance();
    const rt::Rect thumb = thumbRect();
    ctx.fill.SetColor(theme.scrollbarThumb);
    ctx.fill.RoundedRectangle(thumb.x + 2.0f, thumb.y, thumb.w - 4.0f, thumb.h, 3.0f, 4, true);
}

void AssetGrid::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    ctx.pushClip(abs);
    ctx.fill.SetColor(theme.inputBg);
    ctx.fill.Rectangle(abs.x, abs.y, abs.w, abs.h, true);

    if (overrideFont_) ctx.pushFont(overrideFont_);
    ctx.font.SetFontSize(theme.fontSize);
    ctx.font.SetBatch(&ctx.text);

    const int count = static_cast<int>(mItems.size());
    if (count > 0)
    {
        const int cols = columns();
        const float pad = mMode == AssetViewMode::Grid ? kPad : 0.0f;
        const float height = cellHeight();
        int firstRow = static_cast<int>((mScroll - pad) / height);
        if (firstRow < 0) firstRow = 0;
        const int lastRow = static_cast<int>((mScroll + viewportHeight()) / height);

        const rt::Rect content{ abs.x, abs.y + top(), abs.w, viewportHeight() };
        ctx.pushClip(content);
        for (int row = firstRow; row <= lastRow; ++row)
        {
            for (int column = 0; column < cols; ++column)
            {
                const int index = row * cols + column;
                if (index >= count) break;
                paintItem(ctx, index, itemRect(index));
            }
        }
        paintBar(ctx);
        ctx.popClip();
    }
    if (mMode == AssetViewMode::Details) paintHeader(ctx);

    if (overrideFont_) ctx.popFont();
    ctx.popClip();
    rt::Widget::paint(ctx);
}

void AssetGrid::onMousePress(rt::MouseEvent& event)
{
    if (!enabled_) return;
    event.consumed = true;
    if (event.button == 0 && overBar(event.x, event.y))
    {
        const rt::Rect thumb = thumbRect();
        mBarDrag = true;
        mBarGrab = thumb.contains(event.x, event.y) ? event.y - thumb.y : thumb.h * 0.5f;
        dragBar(event.y);
        return;
    }
    const int index = indexAt(event.x, event.y);
    mActivate = -1;
    if (event.button == 0)
    {
        mPressed = index;
        setSelected(index);
        if (index < 0) return;
        itemClicked.emit(index);
        if (event.clickCount == 2) mActivate = index;
    }
    else if (event.button == 1)
    {
        if (index >= 0) setSelected(index);
        contextRequested.emit(index, event.x, event.y);
    }
}

void AssetGrid::onMouseRelease(rt::MouseEvent& event)
{
    mBarDrag = false;
    mPressed = -1;
    event.consumed = true;
    const int activate = mActivate;
    mActivate = -1;
    if (activate >= 0 && activate == indexAt(event.x, event.y)) itemActivated.emit(activate);
}

void AssetGrid::onMouseMove(rt::MouseEvent& event)
{
    if (mBarDrag)
    {
        dragBar(event.y);
        event.consumed = true;
        return;
    }
    const int index = indexAt(event.x, event.y);
    if (index != mHovered)
    {
        mHovered = index;
        markDirty();
    }
}

void AssetGrid::onMouseLeave()
{
    rt::Widget::onMouseLeave();
    if (mHovered >= 0)
    {
        mHovered = -1;
        markDirty();
    }
}

void AssetGrid::onMouseScroll(rt::MouseEvent& event)
{
    const float step = mMode == AssetViewMode::Grid ? cellHeight() * 0.5f : kRowHeight * 3.0f;
    setScroll(mScroll - event.scrollY * step);
    event.consumed = true;
}

bool AssetGrid::isDragSource() const
{
    return enabled_ && !mBarDrag && mHovered >= 0 && mHovered < static_cast<int>(mItems.size()) &&
           !mItems[static_cast<std::size_t>(mHovered)].directory;
}

rt::DragPayload AssetGrid::onDragBegin()
{
    rt::DragPayload payload;
    mActivate = -1;
    mDragged = mPressed >= 0 ? mPressed : mHovered;
    if (mDragged < 0 || mDragged >= static_cast<int>(mItems.size()) ||
            mItems[static_cast<std::size_t>(mDragged)].directory)
    {
        mDragged = -1;
        return payload;
    }
    payload.type = kDragType;
    payload.data = mItems[static_cast<std::size_t>(mDragged)].path;
    return payload;
}

void AssetGrid::onDragEnd(bool accepted)
{
    if (mDragged >= 0 && mDragged < static_cast<int>(mItems.size()))
    {
        const ct::String path = mItems[static_cast<std::size_t>(mDragged)].path;
        mDragged = -1;
        dragFinished.emit(path, accepted);
    }
    mDragged = -1;
}

bool AssetGrid::payloadPath(const rt::DragPayload& payload, ct::String& path)
{
    if (payload.type != kDragType) return false;
    const ct::String* value = std::any_cast<ct::String>(&payload.data);
    if (!value) return false;
    path = *value;
    return true;
}

} // namespace prisma3d
