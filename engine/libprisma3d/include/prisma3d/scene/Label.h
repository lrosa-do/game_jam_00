#pragma once

#include "prisma3d/scene/Control.h"

#include <ct/string.hpp>

namespace prisma3d
{

class Label : public Control
{
public:
    PRISMA3D_NODE_CLASS(Label, Control);
    static void bindProperties();

    enum class HorizontalAlignment : std::uint8_t
    {
        Left,
        Center,
        Right
    };

    enum class VerticalAlignment : std::uint8_t
    {
        Top,
        Center,
        Bottom
    };

    Label();
    ~Label() override;

    void setText(const ct::String& text);
    const ct::String& getText() const;
    void setFontSize(float size);
    float getFontSize() const;
    void setFontColor(const Math::Vec4& color);
    Math::Vec4 getFontColor() const;
    void setHorizontalAlignment(HorizontalAlignment alignment);
    HorizontalAlignment getHorizontalAlignment() const;
    void setVerticalAlignment(VerticalAlignment alignment);
    VerticalAlignment getVerticalAlignment() const;

    Math::Vec2 getMinimumSize() const override;

protected:
    void draw(Gui& gui) override;

private:
    ct::String mText;
    float mFontSize;
    Math::Vec4 mFontColor;
    HorizontalAlignment mHorizontal;
    VerticalAlignment mVertical;
};

} // namespace prisma3d
