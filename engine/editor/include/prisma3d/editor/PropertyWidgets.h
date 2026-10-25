#pragma once

#include "mathc.h"

#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>
#include <igui/widgets/Widget.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class ColorPopup;

class NameLabel : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(NameLabel, ig::retained::Widget);

    explicit NameLabel(const ct::String& text = ct::String());

    const ct::String& text() const;
    void setText(const ct::String& text);

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;

private:
    ct::String mText;
};

class CategoryHeader : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(CategoryHeader, ig::retained::Widget);

    static constexpr float kIconSize = 16.0f;
    static constexpr float kIconGap = 6.0f;

    explicit CategoryHeader(const ct::String& text = ct::String(),
            ig::retained::IconId icon = ig::retained::IconId::None);

    const ct::String& text() const;
    ig::retained::IconId icon() const;

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;

private:
    ct::String mText;
    ig::retained::IconId mIcon;
};

class RevertButton : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(RevertButton, ig::retained::Widget);

    static constexpr float kSize = 18.0f;

    RevertButton();

    bool active() const;
    void setActive(bool active);

    ig::retained::Signal<> reverted;

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;
    void onMouseRelease(ig::retained::MouseEvent& event) override;

private:
    bool mActive;
};

class CommitInput : public ig::retained::TextInput
{
public:
    IG_WIDGET_CLASS(CommitInput, ig::retained::TextInput);

    CommitInput();

    const ct::String& committedText() const;
    void setCommittedText(const ct::String& text);
    bool pending() const;

    ig::retained::Signal<const ct::String&> committed;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    void onFocusLost() override;

private:
    void commit();

    ct::String mCommitted;
};

class LayerGrid : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(LayerGrid, ig::retained::Widget);

    static constexpr int kMaxLayers = 62;

    LayerGrid(int count, int groupSize);

    int count() const;
    int groupSize() const;
    int visibleCount() const;
    int expansionRows() const;

    bool expanded() const;
    void setExpanded(bool expanded);
    bool expandable() const;

    bool isChecked(int index) const;
    void setChecked(int index, bool checked);
    std::int64_t bits() const;
    void setBits(std::int64_t bits);

    ig::retained::Rect cellRect(int index) const;
    ig::retained::Rect expandRect() const;
    int cellAt(float x, float y) const;
    bool overExpand(float x, float y) const;
    int hovered() const;
    bool expandHovered() const;
    static ct::String layerTooltip(int index);
    static ct::String cellLabel(int index);
    static float labelX(const ig::retained::Rect& cell, float textWidth);
    static float labelBaseline(const ig::retained::Rect& cell);
    static float cellAlpha(bool on, bool hovered);
    static float textAlpha(bool on);

    ig::retained::Signal<std::int64_t> changed;

    Vec2f sizeHint() const override;
    void layout() override;
    void paint(ig::retained::PaintContext& ctx) override;
    void onMousePress(ig::retained::MouseEvent& event) override;
    void onMouseRelease(ig::retained::MouseEvent& event) override;
    void onMouseMove(ig::retained::MouseEvent& event) override;
    void onMouseLeave() override;

private:
    struct Layout
    {
        ig::retained::Rect cells[kMaxLayers];
        ig::retained::Rect arrow;
        int visible = 0;
        int expansionRows = 0;
        bool arrowShown = false;
        float gridHeight = 0.0f;
        float rowHeight = 0.0f;
    };

    void computeLayout(Layout& layout, float width) const;
    float hintWidth() const;
    void updateHovered(float x, float y);
    void updateFlag(bool replace);
    void setHovered(int index, bool expandHovered);

    std::int64_t mBits;
    int mCount;
    int mGroupSize;
    int mHovered;
    float mWidths[2];
    int mHintedRows;
    bool mExpanded;
    bool mExpandHovered;
    bool mDragging;
    bool mDraggingValue;
};

class ColorSwatch : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(ColorSwatch, ig::retained::Widget);

    ColorSwatch();
    ~ColorSwatch() override;

    const Math::Vec4& color() const;
    void setColor(const Math::Vec4& color);
    bool alphaEnabled() const;
    void setAlphaEnabled(bool enabled);

    bool isOpen() const;
    void open();
    void close();
    ig::retained::ColorPicker* picker() const;

    ig::retained::Signal<const Math::Vec4&> colorEdited;

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;

private:
    friend class ColorPopup;

    struct Link
    {
        ColorSwatch* owner;
    };

    void pickerChanged(const ig::retained::Color& color);
    void popupClosed(ColorPopup* popup);

    ct::Rc<Link> mLink;
    Math::Vec4 mColor;
    ColorPopup* mPopup;
    bool mAlpha;
};

} // namespace prisma3d
