#include "prisma3d/editor/ImportDialog.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kAxisTags[3] = { "X", "Y", "Z" };

const rt::Color kAxisColors[3] = { rt::Color(230, 100, 100, 255), rt::Color(120, 210, 120, 255),
    rt::Color(110, 150, 240, 255) };

} // namespace

ImportDialog::ImportDialog(const ct::String& summary)
        : rt::FloatWindow(kTitle),
          mInstantiate(nullptr),
          mTranslate{},
          mRotate{},
          mScale(nullptr),
          mAccept(nullptr),
          mCancel(nullptr),
          mClosed(false)
{
    setResizable(false);
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(8.0f);
    main->setPadding(12.0f);

    main->createChild<rt::Label>(summary);
    mInstantiate = main->createChild<rt::CheckBox>(kInstantiateText);
    mInstantiate->setChecked(true);

    vectorRow(main, "Translate", mTranslate, 0.01f);
    vectorRow(main, "Rotate", mRotate, 0.5f);

    main->createChild<rt::Label>("Scale");
    mScale = main->createChild<NumberField>();
    mScale->setDragSensitivity(0.01f);
    mScale->setValue(1.0f);

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(8.0f);
    buttons->setMainAlign(rt::MainAlign::End);
    mCancel = buttons->createChild<rt::Button>("Cancel");
    mCancel->setSize(90.0f, 0.0f);
    mAccept = buttons->createChild<rt::Button>(kAcceptText);
    mAccept->setSize(90.0f, 0.0f);

    mInstantiate->toggled.connect([this](bool) { syncEnabled(); });
    mAccept->clicked.connect([this] { accept(); });
    mCancel->clicked.connect([this] { cancel(); });
    closed.connect([this] { cancel(); });

    const float width = 360.0f;
    const float height = 400.0f;
    setFloatSize(width, height);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);
}

rt::Widget* ImportDialog::vectorRow(rt::Widget* parent, const char* label, NumberField** fields,
        float sensitivity)
{
    parent->createChild<rt::Label>(label);
    rt::BoxLayout* row = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    row->setPadding(0.0f);
    row->setSpacing(4.0f);
    for (int axis = 0; axis < 3; ++axis)
    {
        fields[axis] = row->createChild<NumberField>(kAxisTags[axis]);
        fields[axis]->setTagColor(kAxisColors[axis]);
        fields[axis]->setDragSensitivity(sensitivity);
        fields[axis]->setStretch(1.0f);
    }
    return row;
}

ImportOptions ImportDialog::options() const
{
    ImportOptions result;
    result.instantiate = mInstantiate->isChecked();
    result.translate =
            Math::Vec3(mTranslate[0]->value(), mTranslate[1]->value(), mTranslate[2]->value());
    result.rotate = Math::Vec3(mRotate[0]->value(), mRotate[1]->value(), mRotate[2]->value());
    result.scale = mScale->value();
    return result;
}

void ImportDialog::setOptions(const ImportOptions& options)
{
    mInstantiate->setChecked(options.instantiate);
    for (int axis = 0; axis < 3; ++axis)
    {
        mTranslate[axis]->setValue(options.translate[axis]);
        mRotate[axis]->setValue(options.rotate[axis]);
    }
    mScale->setValue(options.scale);
    syncEnabled();
}

void ImportDialog::syncEnabled()
{
    const bool enabled = mInstantiate->isChecked();
    for (int axis = 0; axis < 3; ++axis)
    {
        mTranslate[axis]->setEnabled(enabled);
        mRotate[axis]->setEnabled(enabled);
    }
    mScale->setEnabled(enabled);
}

void ImportDialog::accept()
{
    if (mClosed) return;
    ImportOptions chosen = options();
    if (chosen.scale <= 0.0f) chosen.scale = 1.0f;
    close();
    importRequested.emit(chosen);
}

void ImportDialog::cancel()
{
    if (mClosed) return;
    close();
    dismissed.emit();
}

void ImportDialog::close()
{
    mClosed = true;
    rt::WidgetApp::instance().removeFloat(this);
}

rt::CheckBox* ImportDialog::instantiateBox() const
{
    return mInstantiate;
}

NumberField* ImportDialog::translateField(int axis) const
{
    return axis >= 0 && axis < 3 ? mTranslate[axis] : nullptr;
}

NumberField* ImportDialog::rotateField(int axis) const
{
    return axis >= 0 && axis < 3 ? mRotate[axis] : nullptr;
}

NumberField* ImportDialog::scaleField() const
{
    return mScale;
}

rt::Button* ImportDialog::importButton() const
{
    return mAccept;
}

rt::Button* ImportDialog::cancelButton() const
{
    return mCancel;
}

void ImportDialog::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape)
    {
        event.consumed = true;
        cancel();
        return;
    }
    if (event.key == rt::Key::Return || event.key == rt::Key::KPEnter)
    {
        event.consumed = true;
        accept();
        return;
    }
    rt::FloatWindow::onKeyPress(event);
}

} // namespace prisma3d
