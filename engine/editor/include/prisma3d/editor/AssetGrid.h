#pragma once

#include "prisma3d/editor/AssetTypes.h"

#include <igui/widgets/Signal.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

struct AssetEntry
{
    ct::String name;
    ct::String path;
    AssetKind kind = AssetKind::Other;
    std::uint64_t size = 0;
    bool directory = false;
};

struct AssetThumbnail
{
    ig::retained::TextureHandle texture;
    float aspect = 1.0f;
};

enum class AssetViewMode : int
{
    Grid,
    List,
    Details
};

class AssetGrid : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(AssetGrid, ig::retained::Widget);

    static constexpr const char* kDragType = "prisma3d/asset";
    static constexpr float kMinThumbnail = 48.0f;
    static constexpr float kMaxThumbnail = 256.0f;
    static constexpr float kRowHeight = 24.0f;
    static constexpr float kHeaderHeight = 22.0f;

    using ThumbnailProvider = ct::Function<AssetThumbnail(const AssetEntry&)>;

    AssetGrid();

    void setItems(const ct::Vector<AssetEntry>& items);
    std::size_t itemCount() const;
    const AssetEntry& item(std::size_t index) const;

    void setViewMode(AssetViewMode mode);
    AssetViewMode viewMode() const;
    void setThumbnailSize(float size);
    float thumbnailSize() const;
    void setThumbnailProvider(const ThumbnailProvider& provider);

    int selected() const;
    void setSelected(int index);
    int hovered() const;
    int indexAt(float x, float y) const;
    ig::retained::Rect itemRect(int index) const;

    int columns() const;
    float contentHeight() const;
    float scroll() const;
    void setScroll(float value);
    void ensureVisible(int index);

    static bool payloadPath(const ig::retained::DragPayload& payload, ct::String& path);

    ig::retained::Signal<int> itemClicked;
    ig::retained::Signal<int> itemActivated;
    ig::retained::Signal<int, float, float> contextRequested;
    ig::retained::Signal<const ct::String&, bool> dragFinished;

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;
    void onMousePress(ig::retained::MouseEvent& event) override;
    void onMouseRelease(ig::retained::MouseEvent& event) override;
    void onMouseMove(ig::retained::MouseEvent& event) override;
    void onMouseLeave() override;
    void onMouseScroll(ig::retained::MouseEvent& event) override;
    bool isDragSource() const override;
    ig::retained::DragPayload onDragBegin() override;
    void onDragEnd(bool accepted) override;

private:
    float viewportHeight() const;
    float maxScroll() const;
    float top() const;
    float cellWidth() const;
    float cellHeight() const;
    ig::retained::Rect barRect() const;
    ig::retained::Rect thumbRect() const;
    bool overBar(float x, float y) const;
    void dragBar(float y);
    void paintItem(ig::retained::PaintContext& ctx, int index, const ig::retained::Rect& cell);
    void paintGridItem(ig::retained::PaintContext& ctx, const AssetEntry& entry,
            const ig::retained::Rect& cell);
    void paintRowItem(ig::retained::PaintContext& ctx, const AssetEntry& entry,
            const ig::retained::Rect& cell);
    void paintHeader(ig::retained::PaintContext& ctx);
    void paintBar(ig::retained::PaintContext& ctx);

    ct::Vector<AssetEntry> mItems;
    AssetViewMode mMode;
    float mThumbnail;
    float mScroll;
    int mSelected;
    int mHovered;
    int mPressed;
    int mDragged;
    int mActivate;
    bool mBarDrag;
    float mBarGrab;
    ThumbnailProvider mProvider;
};

} // namespace prisma3d
