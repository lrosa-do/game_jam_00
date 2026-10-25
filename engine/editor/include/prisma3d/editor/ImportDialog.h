#pragma once

#include "prisma3d/editor/NumberField.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/string.hpp>

#include "mathc.h"

namespace prisma3d
{

struct ImportOptions
{
    bool instantiate = true;
    Math::Vec3 translate = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 rotate = Math::Vec3(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
};

class ImportDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(ImportDialog, ig::retained::FloatWindow);

    static constexpr const char* kTitle = "Import Model";
    static constexpr const char* kInstantiateText = "Instantiate after import";
    static constexpr const char* kAcceptText = "Import";

    ImportDialog(const ct::String& summary);

    ImportOptions options() const;
    void setOptions(const ImportOptions& options);

    void accept();
    void cancel();

    ig::retained::CheckBox* instantiateBox() const;
    NumberField* translateField(int axis) const;
    NumberField* rotateField(int axis) const;
    NumberField* scaleField() const;
    ig::retained::Button* importButton() const;
    ig::retained::Button* cancelButton() const;

    ig::retained::Signal<const ImportOptions&> importRequested;
    ig::retained::Signal<> dismissed;

    void onKeyPress(ig::retained::KeyEvent& event) override;

private:
    ig::retained::Widget* vectorRow(ig::retained::Widget* parent, const char* label,
            NumberField** fields, float sensitivity);
    void syncEnabled();
    void close();

    ig::retained::CheckBox* mInstantiate;
    NumberField* mTranslate[3];
    NumberField* mRotate[3];
    NumberField* mScale;
    ig::retained::Button* mAccept;
    ig::retained::Button* mCancel;
    bool mClosed;
};

} // namespace prisma3d
