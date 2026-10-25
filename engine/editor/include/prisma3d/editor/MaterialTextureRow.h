#pragma once

#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/render/MaterialStore.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

class TextureThumb : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(TextureThumb, ig::retained::Widget);

    static constexpr float kSize = 34.0f;

    TextureId texture() const;
    void setTexture(TextureId texture);

    Vec2f sizeHint() const override;
    void paint(ig::retained::PaintContext& ctx) override;

private:
    TextureId mTexture;
};

class MaterialTextureRow : public ig::retained::BoxLayout
{
public:
    IG_WIDGET_CLASS(MaterialTextureRow, ig::retained::BoxLayout);

    static constexpr const char* kEmptyText = "<empty>";
    static constexpr const char* kBrowseTooltip = "Choose an image from the project";
    static constexpr const char* kClearTooltip = "Clear the texture";

    MaterialTextureRow(const ct::String& label, MaterialTexture slot);

    MaterialTexture slot() const;
    const ct::String& path() const;
    void setPath(const ct::String& path, TextureId texture);
    void setRevertible(bool revertible);

    static ct::String fileNameOf(const ct::String& path);
    static bool acceptsPath(const ct::String& path);

    TextureThumb* thumb() const;
    NameLabel* nameLabel() const;
    ig::retained::Button* browseButton() const;
    ig::retained::Button* clearButton() const;
    RevertButton* revertButton() const;

    ig::retained::Signal<const ct::String&> edited;
    ig::retained::Signal<> browseRequested;
    ig::retained::Signal<> revertRequested;

    bool acceptsDrop(const ig::retained::DragPayload& payload) override;
    void onDropReceive(const ig::retained::DragPayload& payload) override;

private:
    MaterialTexture mSlot;
    ct::String mPath;
    NameLabel* mLabel;
    RevertButton* mRevert;
    TextureThumb* mThumb;
    NameLabel* mName;
    ig::retained::Button* mBrowse;
    ig::retained::Button* mClear;
};

} // namespace prisma3d
