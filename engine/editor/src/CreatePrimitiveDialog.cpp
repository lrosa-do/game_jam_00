#include "prisma3d/editor/CreatePrimitiveDialog.h"

#include "prisma3d/editor/PrimitiveParameterForm.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/scene/Node.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kWidth = 360.0f;
const float kChrome = 96.0f;
const float kRowHeight = 32.0f;
const float kGap = 8.0f;
const float kPadding = 10.0f;
const float kButtonWidth = 90.0f;

} // namespace

CreatePrimitiveDialog::CreatePrimitiveDialog(PrimitiveKind kind, const ct::String& label,
        const PrimitiveRecipe& initial)
        : rt::FloatWindow(ct::String(kTitlePrefix) + label),
          mKind(kind),
          mLabel(label),
          mName(nullptr),
          mParameters(nullptr),
          mSummary(nullptr),
          mCreate(nullptr),
          mCancel(nullptr),
          mClosed(false)
{
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(kGap);
    main->setPadding(kPadding);

    rt::BoxLayout* nameRow = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    nameRow->setPadding(0.0f);
    nameRow->setSpacing(4.0f);
    NameLabel* nameLabel = nameRow->createChild<NameLabel>("Name");
    nameLabel->setStretch(4.0f);
    nameRow->createChild<RevertButton>();
    mName = nameRow->createChild<rt::TextInput>(label);
    mName->setStretch(6.0f);
    mName->setPlaceholder(kNamePlaceholder);

    mParameters = main->createChild<PrimitiveParameterForm>();
    PrimitiveRecipe start = initial;
    start.kind = kind;
    mParameters->setRecipe(start);

    mSummary = main->createChild<rt::Label>(ct::String());
    refreshSummary();

    rt::BoxLayout* spacer = main->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    spacer->setPadding(0.0f);
    spacer->setStretch(1.0f);

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(kGap);
    buttons->setMainAlign(rt::MainAlign::End);
    mCancel = buttons->createChild<rt::Button>(kCancelText);
    mCancel->setSize(kButtonWidth, 0.0f);
    mCreate = buttons->createChild<rt::Button>(kCreateText);
    mCreate->setSize(kButtonWidth, 0.0f);

    mName->submitted.connect([this](const ct::String&) { accept(); });
    mParameters->changed.connect([this] { refreshSummary(); });
    mParameters->editFinished.connect(
            [this]
            {
                rt::WidgetApp& app = rt::WidgetApp::instance();
                if (!app.focused()) app.setFocused(this);
            });
    mCreate->clicked.connect([this] { accept(); });
    mCancel->clicked.connect([this] { cancel(); });
    closed.connect([this] { cancel(); });

    const float rows = static_cast<float>(2 + primitiveParamCount(kind));
    const float height = kChrome + rows * kRowHeight;
    setFloatSize(kWidth, height);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - kWidth) * 0.5f, (io.displayHeight - height) * 0.5f);
}

ct::String CreatePrimitiveDialog::summaryText(std::size_t triangles)
{
    ct::String text;
    text.append_number(static_cast<unsigned long long>(triangles));
    text += triangles == 1 ? " triangle" : " triangles";
    return text;
}

PrimitiveKind CreatePrimitiveDialog::kind() const
{
    return mKind;
}

const ct::String& CreatePrimitiveDialog::label() const
{
    return mLabel;
}

ct::String CreatePrimitiveDialog::nameText() const
{
    return mName->text();
}

void CreatePrimitiveDialog::setNameText(const ct::String& text)
{
    mName->setText(text);
}

const PrimitiveRecipe& CreatePrimitiveDialog::recipe() const
{
    return mParameters->recipe();
}

ct::String CreatePrimitiveDialog::recipePath() const
{
    return mParameters->path();
}

std::size_t CreatePrimitiveDialog::triangleCount() const
{
    return primitiveTriangleCount(mParameters->recipe());
}

void CreatePrimitiveDialog::refreshSummary()
{
    mSummary->setText(summaryText(triangleCount()));
}

void CreatePrimitiveDialog::accept()
{
    if (mClosed) return;
    ct::String name = Node::validateNodeName(mName->text()).trimmed();
    if (name.empty()) name = mLabel;
    const PrimitiveRecipe values = mParameters->recipe();
    close();
    created.emit(name, values);
}

void CreatePrimitiveDialog::cancel()
{
    if (mClosed) return;
    close();
}

void CreatePrimitiveDialog::close()
{
    mClosed = true;
    rt::WidgetApp::instance().removeFloat(this);
}

rt::TextInput* CreatePrimitiveDialog::nameInput() const
{
    return mName;
}

PrimitiveParameterForm* CreatePrimitiveDialog::parameters() const
{
    return mParameters;
}

rt::Label* CreatePrimitiveDialog::summaryLabel() const
{
    return mSummary;
}

rt::Button* CreatePrimitiveDialog::createButton() const
{
    return mCreate;
}

rt::Button* CreatePrimitiveDialog::cancelButton() const
{
    return mCancel;
}

rt::Widget* CreatePrimitiveDialog::focusOnOpen()
{
    mName->selectAll();
    return mName;
}

void CreatePrimitiveDialog::onKeyPress(rt::KeyEvent& event)
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
