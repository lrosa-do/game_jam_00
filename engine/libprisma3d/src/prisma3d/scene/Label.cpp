#include "prisma3d/scene/Label.h"

#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{


Label::Label()
        : mText(),
          mFontSize(16.0f),
          mFontColor(1.0f, 1.0f, 1.0f, 1.0f),
          mHorizontal(HorizontalAlignment::Left),
          mVertical(VerticalAlignment::Top)
{
}

Label::~Label()
{
}

void Label::setText(const ct::String& text)
{
    mText = text;
    updateMinimumSize();
}

const ct::String& Label::getText() const
{
    return mText;
}

void Label::setFontSize(float size)
{
    mFontSize = size < 1.0f ? 1.0f : size;
    updateMinimumSize();
}

float Label::getFontSize() const
{
    return mFontSize;
}

void Label::setFontColor(const Math::Vec4& color)
{
    mFontColor = color;
}

Math::Vec4 Label::getFontColor() const
{
    return mFontColor;
}

void Label::setHorizontalAlignment(HorizontalAlignment alignment)
{
    mHorizontal = alignment;
}

Label::HorizontalAlignment Label::getHorizontalAlignment() const
{
    return mHorizontal;
}

void Label::setVerticalAlignment(VerticalAlignment alignment)
{
    mVertical = alignment;
}

Label::VerticalAlignment Label::getVerticalAlignment() const
{
    return mVertical;
}

Math::Vec2 Label::getMinimumSize() const
{
    const Gui* gui = Gui::singleton();
    if (!gui || mText.empty()) return Math::Vec2(0.0f, mText.empty() ? 0.0f : mFontSize);
    const ig::Vec2 size = gui->hudTextSize(ig::StringView(mText.c_str(), mText.size()), mFontSize);
    return Math::Vec2(size.x, size.y);
}

void Label::draw(Gui& gui)
{
    if (mText.empty()) return;
    const ig::StringView text(mText.c_str(), mText.size());
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    const float fontSize = mFontSize * drawScale();
    const ig::Vec2 textSize = gui.hudTextSize(text, fontSize);
    float x = origin.x;
    float y = origin.y;
    if (mHorizontal == HorizontalAlignment::Center) x += (size.x - textSize.x) * 0.5f;
    else if (mHorizontal == HorizontalAlignment::Right)
        x += size.x - textSize.x;
    if (mVertical == VerticalAlignment::Center) y += (size.y - textSize.y) * 0.5f;
    else if (mVertical == VerticalAlignment::Bottom)
        y += size.y - textSize.y;
    const Math::Vec4 modulate = getEffectiveModulate();
    const Math::Vec4 color(mFontColor.x * modulate.x, mFontColor.y * modulate.y,
            mFontColor.z * modulate.z, mFontColor.w * modulate.w);
    gui.hudText(text, x, y, Control::guiColor(color), fontSize);
}

void Label::bindProperties()
{
    using DB = NodeClassDB;
    using P = Label;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "text"), &P::setText, &P::getText);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "font_size", 1.0, 512.0, 1.0),
            &P::setFontSize, &P::getFontSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "font_color"), &P::setFontColor,
            &P::getFontColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "horizontal_alignment", PropertyHint::Enum,
                               "Left,Center,Right"),
            &P::setHorizontalAlignment, &P::getHorizontalAlignment);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "vertical_alignment", PropertyHint::Enum,
                               "Top,Center,Bottom"),
            &P::setVerticalAlignment, &P::getVerticalAlignment);
}

} // namespace prisma3d
