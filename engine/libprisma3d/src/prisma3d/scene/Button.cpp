#include "prisma3d/scene/Button.h"

#include "prisma3d/Input.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ScriptInstance.h"

namespace prisma3d
{

namespace
{

const float kPaddingX = 16.0f;
const float kPaddingY = 8.0f;

} // namespace

Button::Button()
        : mText(),
          mFontSize(20.0f),
          mFontColor(1.0f, 1.0f, 1.0f, 1.0f),
          mNormalColor(0.18f, 0.2f, 0.26f, 1.0f),
          mHoverColor(0.28f, 0.32f, 0.42f, 1.0f),
          mPressedColor(0.1f, 0.12f, 0.16f, 1.0f),
          mDisabled(false),
          mHovering(false),
          mPressAttempt(false)
{
    setProcess(true);
    updateMinimumSize();
}

Button::~Button()
{
}

void Button::setText(const ct::String& text)
{
    mText = text;
    updateMinimumSize();
}

const ct::String& Button::getText() const
{
    return mText;
}

void Button::setFontSize(float size)
{
    mFontSize = size < 1.0f ? 1.0f : size;
    updateMinimumSize();
}

float Button::getFontSize() const
{
    return mFontSize;
}

void Button::setFontColor(const Math::Vec4& color)
{
    mFontColor = color;
}

Math::Vec4 Button::getFontColor() const
{
    return mFontColor;
}

void Button::setNormalColor(const Math::Vec4& color)
{
    mNormalColor = color;
}

Math::Vec4 Button::getNormalColor() const
{
    return mNormalColor;
}

void Button::setHoverColor(const Math::Vec4& color)
{
    mHoverColor = color;
}

Math::Vec4 Button::getHoverColor() const
{
    return mHoverColor;
}

void Button::setPressedColor(const Math::Vec4& color)
{
    mPressedColor = color;
}

Math::Vec4 Button::getPressedColor() const
{
    return mPressedColor;
}

void Button::setDisabled(bool disabled)
{
    mDisabled = disabled;
    if (disabled)
    {
        mHovering = false;
        mPressAttempt = false;
    }
}

bool Button::isDisabled() const
{
    return mDisabled;
}

bool Button::isHovered() const
{
    return mHovering;
}

bool Button::isPressed() const
{
    return mPressAttempt && mHovering;
}

Math::Vec2 Button::getMinimumSize() const
{
    const Gui* gui = Gui::singleton();
    Math::Vec2 text(0.0f, mFontSize);
    if (gui && !mText.empty())
    {
        const ig::Vec2 size =
                gui->hudTextSize(ig::StringView(mText.c_str(), mText.size()), mFontSize);
        text = Math::Vec2(size.x, size.y);
    }
    return Math::Vec2(text.x + kPaddingX * 2.0f, text.y + kPaddingY * 2.0f);
}

bool Button::containsPoint(const Math::Vec2& point) const
{
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    return point.x >= origin.x && point.y >= origin.y && point.x < origin.x + size.x &&
           point.y < origin.y + size.y;
}

void Button::press()
{
    if (mDisabled) return;
    for (Node* node = this; node; node = node->getParent())
    {
        ScriptInstance* script = node->getScriptInstance();
        if (!script || !script->hasScript()) continue;
        script->buttonPressed(this);
        return;
    }
}

void Button::update()
{
    const Input* input = Input::singleton();
    if (!input || mDisabled || !isVisibleInTree())
    {
        mHovering = false;
        mPressAttempt = false;
        return;
    }
    const bool inside = containsPoint(input->getMousePosition());
    mHovering = inside;
    if (inside && input->isMouseButtonJustPressed(MouseButton::Left)) mPressAttempt = true;
    if (mPressAttempt && input->isMouseButtonJustReleased(MouseButton::Left))
    {
        mPressAttempt = false;
        if (inside) press();
    }
    if (!input->isMouseButtonPressed(MouseButton::Left)) mPressAttempt = false;
}

void Button::notificationHandler(int what)
{
    Control::notificationHandler(what);
    if (what == kNotificationProcess) update();
}

void Button::draw(Gui& gui)
{
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    const float fontSize = mFontSize * drawScale();
    if (size.x <= 0.0f || size.y <= 0.0f) return;
    const Math::Vec4 modulate = getEffectiveModulate();
    const float dim = mDisabled ? 0.5f : 1.0f;
    const Math::Vec4& background = isPressed() ? mPressedColor
                                   : mHovering ? mHoverColor
                                               : mNormalColor;
    const auto tint = [&](const Math::Vec4& color)
    {
        return Control::guiColor(Math::Vec4(color.x * modulate.x * dim, color.y * modulate.y * dim,
                color.z * modulate.z * dim, color.w * modulate.w));
    };
    const ig::Rect rect(origin.x, origin.y, size.x, size.y);
    gui.hudRectFilled(rect, tint(background));
    gui.hudRectOutline(rect,
            tint(Math::Vec4(background.x + 0.15f, background.y + 0.15f, background.z + 0.15f,
                    background.w)),
            1.0f);
    if (mText.empty()) return;
    const ig::StringView text(mText.c_str(), mText.size());
    const ig::Vec2 textSize = gui.hudTextSize(text, fontSize);
    gui.hudText(text, origin.x + (size.x - textSize.x) * 0.5f,
            origin.y + (size.y - textSize.y) * 0.5f, tint(mFontColor), fontSize);
}

void Button::bindProperties()
{
    using DB = NodeClassDB;
    using P = Button;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "text"), &P::setText, &P::getText);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "font_size", 1.0, 512.0, 1.0),
            &P::setFontSize, &P::getFontSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "font_color"), &P::setFontColor,
            &P::getFontColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "normal_color"), &P::setNormalColor,
            &P::getNormalColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "hover_color"), &P::setHoverColor,
            &P::getHoverColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "pressed_color"), &P::setPressedColor,
            &P::getPressedColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "disabled"), &P::setDisabled,
            &P::isDisabled);
}

} // namespace prisma3d
