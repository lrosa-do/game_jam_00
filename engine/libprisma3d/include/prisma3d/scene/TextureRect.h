#pragma once

#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Control.h"

namespace prisma3d
{

class TextureRect : public Control
{
public:
    PRISMA3D_NODE_CLASS(TextureRect, Control);
    static void bindProperties();

    enum class StretchMode : std::uint8_t
    {
        Scale,
        Tile,
        Keep,
        KeepCentered,
        KeepAspect,
        KeepAspectCentered,
        KeepAspectCovered
    };

    TextureRect();
    ~TextureRect() override;

    void setTexture(TextureId texture);
    TextureId getTexture() const;
    void setStretchMode(StretchMode mode);
    StretchMode getStretchMode() const;
    void setExpand(bool expand);
    bool isExpand() const;
    void setFlipH(bool flip);
    bool isFlipH() const;
    void setFlipV(bool flip);
    bool isFlipV() const;

    Math::Vec2 getMinimumSize() const override;

protected:
    void draw(Gui& gui) override;

private:
    Math::Vec2 textureSize() const;

    TextureId mTexture;
    StretchMode mStretch;
    bool mExpand;
    bool mFlipH;
    bool mFlipV;
};

} // namespace prisma3d
