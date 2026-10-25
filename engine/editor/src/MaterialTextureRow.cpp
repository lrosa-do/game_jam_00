#include "prisma3d/editor/MaterialTextureRow.h"

#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetTypes.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/render/TextureStore.h"

#include <igui/widgets/Theme.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kSpacing = 4.0f;
const float kButtonWidth = 28.0f;

} // namespace

TextureId TextureThumb::texture() const
{
    return mTexture;
}

void TextureThumb::setTexture(TextureId texture)
{
    if (mTexture == texture) return;
    mTexture = texture;
    markDirty();
}

rt::Widget::Vec2f TextureThumb::sizeHint() const
{
    return { kSize, kSize };
}

void TextureThumb::paint(rt::PaintContext& ctx)
{
    if (!visible_) return;
    const rt::Rect abs = absoluteRect();
    if (ctx.isClipped(abs)) return;

    const rt::Theme& theme = rt::Theme::instance();
    ctx.fill.SetColor(theme.inputBg);
    ctx.fill.Rectangle(abs.x, abs.y, abs.w, abs.h, true);
    TextureStore* store = TextureStore::singleton();
    if (mTexture.valid() && store && store->valid(mTexture))
        ctx.drawImage(WidgetHost::textureHandle(mTexture), abs);
    ctx.line.SetColor(theme.borderColor);
    ctx.line.Rectangle(abs.x, abs.y, abs.w, abs.h, false);
}

MaterialTextureRow::MaterialTextureRow(const ct::String& label, MaterialTexture slot)
        : rt::BoxLayout(rt::LayoutDir::Horizontal),
          mSlot(slot)
{
    setPadding(0.0f);
    setSpacing(kSpacing);

    mLabel = createChild<NameLabel>(label);
    mLabel->setStretch(4.0f);
    mRevert = createChild<RevertButton>();
    mRevert->reverted.connect([this] { revertRequested.emit(); });

    rt::BoxLayout* box = createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    box->setPadding(0.0f);
    box->setSpacing(kSpacing);
    box->setStretch(6.0f);

    mThumb = box->createChild<TextureThumb>();
    mThumb->setLayoutAlign(rt::Align::Center);
    mName = box->createChild<NameLabel>(ct::String(kEmptyText));
    mName->setStretch(1.0f);
    mName->setLayoutAlign(rt::Align::Center);
    mBrowse = box->createChild<rt::Button>();
    mBrowse->setIconId(EditorIcons::get("Load"));
    mBrowse->setSize(kButtonWidth, 0.0f);
    mBrowse->setTooltip(kBrowseTooltip);
    mBrowse->setLayoutAlign(rt::Align::Center);
    mBrowse->clicked.connect([this] { browseRequested.emit(); });
    mClear = box->createChild<rt::Button>();
    mClear->setIconId(EditorIcons::get("Close"));
    mClear->setSize(kButtonWidth, 0.0f);
    mClear->setTooltip(kClearTooltip);
    mClear->setLayoutAlign(rt::Align::Center);
    mClear->clicked.connect(
            [this]
            {
                if (!mPath.empty()) edited.emit(ct::String());
            });
}

MaterialTexture MaterialTextureRow::slot() const
{
    return mSlot;
}

const ct::String& MaterialTextureRow::path() const
{
    return mPath;
}

void MaterialTextureRow::setPath(const ct::String& path, TextureId texture)
{
    mPath = path;
    mThumb->setTexture(path.empty() ? TextureId() : texture);
    mName->setText(path.empty() ? ct::String(kEmptyText) : fileNameOf(path));
    mName->setTooltip(path);
    mClear->setEnabled(!path.empty());
}

void MaterialTextureRow::setRevertible(bool revertible)
{
    mRevert->setActive(revertible);
}

ct::String MaterialTextureRow::fileNameOf(const ct::String& path)
{
    const std::size_t slash = path.rfind('/');
    return slash == ct::String::npos ? path : path.substr(slash + 1);
}

bool MaterialTextureRow::acceptsPath(const ct::String& path)
{
    return AssetTypes::classify(path, false) == AssetKind::Texture;
}

TextureThumb* MaterialTextureRow::thumb() const
{
    return mThumb;
}

NameLabel* MaterialTextureRow::nameLabel() const
{
    return mName;
}

rt::Button* MaterialTextureRow::browseButton() const
{
    return mBrowse;
}

rt::Button* MaterialTextureRow::clearButton() const
{
    return mClear;
}

RevertButton* MaterialTextureRow::revertButton() const
{
    return mRevert;
}

bool MaterialTextureRow::acceptsDrop(const rt::DragPayload& payload)
{
    ct::String path;
    return isEnabledInTree() && AssetGrid::payloadPath(payload, path) && acceptsPath(path);
}

void MaterialTextureRow::onDropReceive(const rt::DragPayload& payload)
{
    ct::String path;
    if (AssetGrid::payloadPath(payload, path) && acceptsPath(path)) edited.emit(path);
}

} // namespace prisma3d
