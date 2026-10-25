#pragma once

#include "prisma3d/scene/Control.h"

namespace prisma3d
{

class ColorRect : public Control
{
public:
    PRISMA3D_NODE_CLASS(ColorRect, Control);
    static void bindProperties();

    ColorRect();
    ~ColorRect() override;

    void setColor(const Math::Vec4& color);
    Math::Vec4 getColor() const;

protected:
    void draw(Gui& gui) override;

private:
    Math::Vec4 mColor;
};

} // namespace prisma3d
