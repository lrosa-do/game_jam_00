#include "prisma3d/scene/ColorRect.h"

#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

ColorRect::ColorRect()
        : mColor(1.0f, 1.0f, 1.0f, 1.0f)
{
}

ColorRect::~ColorRect()
{
}

void ColorRect::setColor(const Math::Vec4& color)
{
    mColor = color;
}

Math::Vec4 ColorRect::getColor() const
{
    return mColor;
}

void ColorRect::draw(Gui& gui)
{
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    if (size.x <= 0.0f || size.y <= 0.0f) return;
    const Math::Vec4 modulate = getEffectiveModulate();
    gui.hudRectFilled(ig::Rect(origin.x, origin.y, size.x, size.y),
            Control::guiColor(Math::Vec4(mColor.x * modulate.x, mColor.y * modulate.y,
                    mColor.z * modulate.z, mColor.w * modulate.w)));
}

void ColorRect::bindProperties()
{
    using DB = NodeClassDB;
    using P = ColorRect;
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "color"), &P::setColor, &P::getColor);
}

} // namespace prisma3d
