#pragma once

#include "prisma3d/scene/Control.h"

#include <ct/string.hpp>

namespace prisma3d
{

class Button : public Control
{
public:
    PRISMA3D_NODE_CLASS(Button, Control);
    static void bindProperties();

    Button();
    ~Button() override;

    void setText(const ct::String& text);
    const ct::String& getText() const;
    void setFontSize(float size);
    float getFontSize() const;
    void setFontColor(const Math::Vec4& color);
    Math::Vec4 getFontColor() const;
    void setNormalColor(const Math::Vec4& color);
    Math::Vec4 getNormalColor() const;
    void setHoverColor(const Math::Vec4& color);
    Math::Vec4 getHoverColor() const;
    void setPressedColor(const Math::Vec4& color);
    Math::Vec4 getPressedColor() const;
    void setDisabled(bool disabled);
    bool isDisabled() const;

    bool isHovered() const;
    bool isPressed() const;
    void press();

    Math::Vec2 getMinimumSize() const override;

protected:
    void notificationHandler(int what) override;
    void draw(Gui& gui) override;

private:
    bool containsPoint(const Math::Vec2& point) const;
    void update();

    ct::String mText;
    float mFontSize;
    Math::Vec4 mFontColor;
    Math::Vec4 mNormalColor;
    Math::Vec4 mHoverColor;
    Math::Vec4 mPressedColor;
    bool mDisabled;
    bool mHovering;
    bool mPressAttempt;
};

} // namespace prisma3d
