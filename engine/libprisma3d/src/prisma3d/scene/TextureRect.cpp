#include "prisma3d/scene/TextureRect.h"

#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"

namespace prisma3d
{

TextureRect::TextureRect()
        : mTexture(),
          mStretch(StretchMode::Scale),
          mExpand(false),
          mFlipH(false),
          mFlipV(false)
{
}

TextureRect::~TextureRect()
{
}

void TextureRect::setTexture(TextureId texture)
{
    mTexture = texture;
    updateMinimumSize();
}

TextureId TextureRect::getTexture() const
{
    return mTexture;
}

void TextureRect::setStretchMode(StretchMode mode)
{
    mStretch = mode;
}

TextureRect::StretchMode TextureRect::getStretchMode() const
{
    return mStretch;
}

void TextureRect::setExpand(bool expand)
{
    mExpand = expand;
    updateMinimumSize();
}

bool TextureRect::isExpand() const
{
    return mExpand;
}

void TextureRect::setFlipH(bool flip)
{
    mFlipH = flip;
}

bool TextureRect::isFlipH() const
{
    return mFlipH;
}

void TextureRect::setFlipV(bool flip)
{
    mFlipV = flip;
}

bool TextureRect::isFlipV() const
{
    return mFlipV;
}

Math::Vec2 TextureRect::textureSize() const
{
    const TextureStore* store = TextureStore::singleton();
    const TextureInfo* info = store && mTexture.valid() ? store->get(mTexture) : nullptr;
    if (!info) return Math::Vec2(0.0f, 0.0f);
    return Math::Vec2(static_cast<float>(info->width), static_cast<float>(info->height));
}

Math::Vec2 TextureRect::getMinimumSize() const
{
    return mExpand ? Math::Vec2(0.0f, 0.0f) : textureSize();
}

void TextureRect::draw(Gui& gui)
{
    const Math::Vec2 scale = getGlobalScale();
    const Math::Vec2 image(textureSize().x * scale.x, textureSize().y * scale.y);
    if (!mTexture.valid() || image.x <= 0.0f || image.y <= 0.0f) return;
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    if (size.x <= 0.0f || size.y <= 0.0f) return;

    float x = origin.x;
    float y = origin.y;
    float width = size.x;
    float height = size.y;
    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 1.0f;
    float v1 = 1.0f;
    switch (mStretch)
    {
        case StretchMode::Scale:
        case StretchMode::Tile:
            break;
        case StretchMode::Keep:
            width = image.x;
            height = image.y;
            break;
        case StretchMode::KeepCentered:
            width = image.x;
            height = image.y;
            x += (size.x - width) * 0.5f;
            y += (size.y - height) * 0.5f;
            break;
        case StretchMode::KeepAspect:
        case StretchMode::KeepAspectCentered:
        {
            const float scale =
                    size.x / image.x < size.y / image.y ? size.x / image.x : size.y / image.y;
            width = image.x * scale;
            height = image.y * scale;
            if (mStretch == StretchMode::KeepAspectCentered)
            {
                x += (size.x - width) * 0.5f;
                y += (size.y - height) * 0.5f;
            }
            break;
        }
        case StretchMode::KeepAspectCovered:
        {
            const float scale =
                    size.x / image.x > size.y / image.y ? size.x / image.x : size.y / image.y;
            const float coveredWidth = image.x * scale;
            const float coveredHeight = image.y * scale;
            const float cropX = (coveredWidth - size.x) / coveredWidth * 0.5f;
            const float cropY = (coveredHeight - size.y) / coveredHeight * 0.5f;
            u0 = cropX;
            u1 = 1.0f - cropX;
            v0 = cropY;
            v1 = 1.0f - cropY;
            break;
        }
    }
    if (mFlipH)
    {
        const float swap = u0;
        u0 = u1;
        u1 = swap;
    }
    if (mFlipV)
    {
        const float swap = v0;
        v0 = v1;
        v1 = swap;
    }
    gui.hudImage(mTexture, ig::Rect(x, y, width, height), 0.0f, ig::Vec2(u0, v0), ig::Vec2(u1, v1),
            Control::guiColor(getEffectiveModulate()));
}

void TextureRect::bindProperties()
{
    using DB = NodeClassDB;
    using P = TextureRect;
    resourceprop::add<P, TextureId>(PropertyInfo::resource("texture", "Texture2D"), &P::setTexture,
            &P::getTexture, true);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "stretch_mode", PropertyHint::Enum,
                               "Scale,Tile,Keep,Keep Centered,Keep Aspect,Keep Aspect Centered,"
                               "Keep Aspect Covered"),
            &P::setStretchMode, &P::getStretchMode);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "expand"), &P::setExpand, &P::isExpand);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "flip_h"), &P::setFlipH, &P::isFlipH);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "flip_v"), &P::setFlipV, &P::isFlipV);
}

} // namespace prisma3d
