#pragma once

#include "prisma3d/editor/PrimitiveParameterForm.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/string.hpp>

#include <cstddef>

namespace prisma3d
{

class CreatePrimitiveDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(CreatePrimitiveDialog, ig::retained::FloatWindow);

    static constexpr const char* kTitlePrefix = "Create ";
    static constexpr const char* kCreateText = "Create";
    static constexpr const char* kCancelText = "Cancel";
    static constexpr const char* kNamePlaceholder = "Name";

    CreatePrimitiveDialog(PrimitiveKind kind, const ct::String& label,
            const PrimitiveRecipe& initial);

    static ct::String summaryText(std::size_t triangles);

    PrimitiveKind kind() const;
    const ct::String& label() const;
    ct::String nameText() const;
    void setNameText(const ct::String& text);
    const PrimitiveRecipe& recipe() const;
    ct::String recipePath() const;
    std::size_t triangleCount() const;

    void accept();
    void cancel();

    ig::retained::TextInput* nameInput() const;
    PrimitiveParameterForm* parameters() const;
    ig::retained::Label* summaryLabel() const;
    ig::retained::Button* createButton() const;
    ig::retained::Button* cancelButton() const;

    ig::retained::Signal<const ct::String&, const PrimitiveRecipe&> created;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    ig::retained::Widget* focusOnOpen() override;

private:
    void refreshSummary();
    void close();

    PrimitiveKind mKind;
    ct::String mLabel;
    ig::retained::TextInput* mName;
    PrimitiveParameterForm* mParameters;
    ig::retained::Label* mSummary;
    ig::retained::Button* mCreate;
    ig::retained::Button* mCancel;
    bool mClosed;
};

} // namespace prisma3d
