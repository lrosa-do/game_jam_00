#pragma once

#include "prisma3d/scene/Node.h"

#include "mathc.h"

#include <igui/Color.hpp>

namespace prisma3d
{

class Gui;

class Control : public Node
{
public:
    PRISMA3D_NODE_CLASS(Control, Node);
    static void bindProperties();

    enum Side
    {
        kSideLeft,
        kSideTop,
        kSideRight,
        kSideBottom
    };

    enum class GrowDirection : std::uint8_t
    {
        Begin,
        End,
        Both
    };

    enum class LayoutPreset : std::uint8_t
    {
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight,
        CenterLeft,
        CenterTop,
        CenterRight,
        CenterBottom,
        Center,
        LeftWide,
        TopWide,
        RightWide,
        BottomWide,
        VCenterWide,
        HCenterWide,
        FullRect
    };

    enum class PresetMode : std::uint8_t
    {
        MinSize,
        KeepWidth,
        KeepHeight,
        KeepSize
    };

    static constexpr int kPresetCustom = -1;
    static ig::Color guiColor(const Math::Vec4& color);

    Control();
    ~Control() override;

    void setAnchor(Side side, float anchor, bool keepOffset = false,
            bool pushOppositeAnchor = true);
    float getAnchor(Side side) const;
    void setOffset(Side side, float value);
    float getOffset(Side side) const;
    void setAnchorsPreset(LayoutPreset preset, bool keepOffsets = false);
    void setOffsetsPreset(LayoutPreset preset, PresetMode mode, float margin = 0.0f);
    void setGrowDirectionPreset(LayoutPreset preset);
    void setLayoutPreset(int preset);
    int getLayoutPreset() const;

    void setPosition(const Math::Vec2& position);
    Math::Vec2 getPosition() const;
    void setSize(const Math::Vec2& size);
    Math::Vec2 getSize() const;
    Math::Vec2 getGlobalPosition() const;
    Math::Vec2 getParentSize() const;
    void setScale(const Math::Vec2& scale);
    Math::Vec2 getScale() const;
    void setPivotOffset(const Math::Vec2& pivot);
    Math::Vec2 getPivotOffset() const;
    Math::Vec2 getGlobalScale() const;
    Math::Vec2 toGlobal(const Math::Vec2& local) const;
    Math::Vec2 getDrawSize() const;
    float drawScale() const;

    void setCustomMinimumSize(const Math::Vec2& size);
    Math::Vec2 getCustomMinimumSize() const;
    virtual Math::Vec2 getMinimumSize() const;
    Math::Vec2 getCombinedMinimumSize() const;
    void updateMinimumSize();

    void setHGrowDirection(GrowDirection direction);
    GrowDirection getHGrowDirection() const;
    void setVGrowDirection(GrowDirection direction);
    GrowDirection getVGrowDirection() const;

    void setVisible(bool visible);
    bool isVisible() const;
    bool isVisibleInTree() const;
    void setModulate(const Math::Vec4& color);
    Math::Vec4 getModulate() const;
    Math::Vec4 getEffectiveModulate() const;

    Control* getParentControl() const;

    void setAnchorLeft(float value);
    float getAnchorLeft() const;
    void setAnchorTop(float value);
    float getAnchorTop() const;
    void setAnchorRight(float value);
    float getAnchorRight() const;
    void setAnchorBottom(float value);
    float getAnchorBottom() const;
    void setOffsetLeft(float value);
    float getOffsetLeft() const;
    void setOffsetTop(float value);
    float getOffsetTop() const;
    void setOffsetRight(float value);
    float getOffsetRight() const;
    void setOffsetBottom(float value);
    float getOffsetBottom() const;

protected:
    void notificationHandler(int what) override;
    virtual void draw(Gui& gui);
    void sizeChanged();

private:
    void computeLayoutRect(const Math::Vec2& position, const Math::Vec2& size);

    float mAnchor[4];
    float mOffset[4];
    GrowDirection mHGrow;
    GrowDirection mVGrow;
    int mPreset;
    Math::Vec2 mPosition;
    Math::Vec2 mSize;
    Math::Vec2 mCustomMinimumSize;
    Math::Vec2 mLastParentSize;
    Math::Vec2 mScale;
    Math::Vec2 mPivotOffset;
    Math::Vec4 mModulate;
    bool mVisible;
};

} // namespace prisma3d
