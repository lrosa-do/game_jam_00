#include "prisma3d/editor/CreateTerrainDialog.h"

#include "prisma3d/editor/AssetTypes.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/scene/Node.h"

#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kWidth = 400.0f;
const float kHeight = 330.0f;
const float kGap = 8.0f;
const float kPadding = 10.0f;
const float kButtonWidth = 90.0f;
const float kBrowseWidth = 34.0f;
const std::uint16_t kSizes[CreateTerrainDialog::kSizeCount] = { 129, 257, 513, 1025 };

rt::BoxLayout* addRow(rt::BoxLayout* parent, const char* label, const char* tooltip)
{
    rt::BoxLayout* row = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    row->setPadding(0.0f);
    row->setSpacing(4.0f);
    NameLabel* name = row->createChild<NameLabel>(label);
    name->setStretch(4.0f);
    name->setTooltip(tooltip);
    rt::BoxLayout* editors = row->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    editors->setPadding(0.0f);
    editors->setSpacing(4.0f);
    editors->setStretch(6.0f);
    return editors;
}

} // namespace

CreateTerrainDialog::CreateTerrainDialog(const TerrainCreateSettings& initial,
        const Project* project, EditorSettings* settings)
        : rt::FloatWindow(ct::String(kTitle)),
          mLink(ct::make_rc<Link>(Link{ this })),
          mProject(project),
          mSettings(settings),
          mName(nullptr),
          mSize(nullptr),
          mWorldSize(nullptr),
          mHeightmap(nullptr),
          mBrowse(nullptr),
          mHeightScale(nullptr),
          mFlatHeight(nullptr),
          mError(nullptr),
          mCreate(nullptr),
          mCancel(nullptr),
          mClosed(false)
{
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(kGap);
    main->setPadding(kPadding);

    rt::BoxLayout* nameEditors = addRow(main, "Name", "Name of the new node");
    mName = nameEditors->createChild<rt::TextInput>(ct::String(kDefaultName));
    mName->setStretch(1.0f);

    rt::BoxLayout* sizeEditors =
            addRow(main, "Size", "Vertices along each side; the terrain has size - 1 cells");
    mSize = sizeEditors->createChild<rt::ComboBox>();
    mSize->setStretch(1.0f);
    for (int i = 0; i < kSizeCount; ++i)
    {
        ct::String text;
        text.append_number(static_cast<unsigned long long>(kSizes[i]));
        text += " x ";
        text.append_number(static_cast<unsigned long long>(kSizes[i]));
        mSize->addItem(text);
    }
    mSize->setSelectedIndex(sizeIndex(initial.size));

    mWorldSize = addNumberRow(main, "World Size", "Width of the terrain in metres",
            initial.worldSize, 1.0, 100000.0);

    rt::BoxLayout* heightmapEditors = addRow(main, "Heightmap",
            "Image (8 or 16 bit) or square 16 bit .raw whose brightness gives the height; empty "
            "for flat");
    mHeightmap = heightmapEditors->createChild<CommitInput>();
    mHeightmap->setStretch(1.0f);
    mHeightmap->setCommittedText(initial.heightmap);
    mBrowse = heightmapEditors->createChild<rt::Button>();
    mBrowse->setIconId(EditorIcons::get("Load"));
    mBrowse->setSize(kBrowseWidth, 0.0f);
    mBrowse->setTooltip("Choose an image");

    mHeightScale = addNumberRow(main, "Height Scale", "Height in metres of a white pixel",
            initial.heightScale, 0.0, 100000.0);
    mFlatHeight = addNumberRow(main, "Flat Height", "Height in metres when there is no heightmap",
            initial.flatHeight, -100000.0, 100000.0);

    mError = main->createChild<rt::Label>(ct::String());

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

    const ct::Rc<Link> link = mLink;
    mName->submitted.connect(
            [link](const ct::String&)
            {
                if (link->owner) link->owner->accept();
            });
    mHeightmap->committed.connect(
            [link](const ct::String&)
            {
                if (link->owner) link->owner->syncRows();
            });
    mBrowse->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->browse();
            });
    mCreate->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->accept();
            });
    mCancel->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->cancel();
            });
    closed.connect(
            [link]
            {
                if (link->owner) link->owner->cancel();
            });

    syncRows();
    setFloatSize(kWidth, kHeight);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - kWidth) * 0.5f, (io.displayHeight - kHeight) * 0.5f);
}

CreateTerrainDialog::~CreateTerrainDialog()
{
    mLink->owner = nullptr;
}

NumberField* CreateTerrainDialog::addNumberRow(rt::BoxLayout* parent, const char* label,
        const char* tooltip, float value, double minimum, double maximum)
{
    rt::BoxLayout* editors = addRow(parent, label, tooltip);
    NumberField* field = editors->createChild<NumberField>("");
    field->setStretch(1.0f);
    field->setRange(minimum, maximum);
    field->setValue(value);
    return field;
}

std::uint16_t CreateTerrainDialog::sizeOption(int index)
{
    return index >= 0 && index < kSizeCount ? kSizes[index] : kSizes[2];
}

int CreateTerrainDialog::sizeIndex(std::uint16_t size)
{
    for (int i = 0; i < kSizeCount; ++i)
        if (kSizes[i] == size) return i;
    return 2;
}

ct::String CreateTerrainDialog::nameText() const
{
    return mName->text();
}

TerrainCreateSettings CreateTerrainDialog::settings() const
{
    TerrainCreateSettings result;
    result.size = sizeOption(mSize->selectedIndex());
    result.worldSize = mWorldSize->value();
    result.heightmap = mHeightmap->committedText().trimmed();
    result.heightScale = mHeightScale->value();
    result.flatHeight = mFlatHeight->value();
    return result;
}

void CreateTerrainDialog::setHeightmap(const ct::String& path)
{
    mHeightmap->setCommittedText(path);
    syncRows();
}

const ct::String& CreateTerrainDialog::error() const
{
    return mErrorText;
}

void CreateTerrainDialog::syncRows()
{
    const bool image = !mHeightmap->committedText().trimmed().empty();
    mHeightScale->setEnabled(image);
    mFlatHeight->setEnabled(!image);
}

void CreateTerrainDialog::browse()
{
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Choose a heightmap");
    dialog->setMode(rt::FileDialog::Mode::Open);
    dialog->setFilter(kHeightmapFilter);
    FileDialogMemory::prepare(*dialog, mSettings, EditorSettings::kFolderTexture, mProject);
    const ct::Rc<Link> link = mLink;
    EditorSettings* settings = mSettings;
    dialog->accepted.connect(
            [link, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderTexture, path);
                if (link->owner) link->owner->heightmapChosen(path);
            });
}

void CreateTerrainDialog::heightmapChosen(const ct::String& path)
{
    ct::String stored = path;
    if (mProject && mProject->isOpen() && !mProject->relativePath(path, stored))
    {
        mErrorText = "Choose an image inside the project folder.";
        mError->setText(mErrorText);
        return;
    }
    const ct::String extension = AssetTypes::extensionOf(stored);
    if (!AssetTypes::isTextureExtension(extension) && extension != "raw" && extension != "r16")
    {
        mErrorText = "Choose an image file or a 16 bit .raw heightmap.";
        mError->setText(mErrorText);
        return;
    }
    mErrorText = ct::String();
    mError->setText(mErrorText);
    setHeightmap(stored);
}

void CreateTerrainDialog::accept()
{
    if (mClosed) return;
    ct::String name = Node::validateNodeName(mName->text()).trimmed();
    if (name.empty()) name = kDefaultName;
    const TerrainCreateSettings values = settings();
    close();
    created.emit(name, values);
}

void CreateTerrainDialog::cancel()
{
    if (mClosed) return;
    close();
}

void CreateTerrainDialog::close()
{
    mClosed = true;
    rt::WidgetApp::instance().removeFloat(this);
}

rt::TextInput* CreateTerrainDialog::nameInput() const
{
    return mName;
}

rt::ComboBox* CreateTerrainDialog::sizeBox() const
{
    return mSize;
}

NumberField* CreateTerrainDialog::worldSizeField() const
{
    return mWorldSize;
}

CommitInput* CreateTerrainDialog::heightmapInput() const
{
    return mHeightmap;
}

NumberField* CreateTerrainDialog::heightScaleField() const
{
    return mHeightScale;
}

NumberField* CreateTerrainDialog::flatHeightField() const
{
    return mFlatHeight;
}

rt::Button* CreateTerrainDialog::createButton() const
{
    return mCreate;
}

rt::Button* CreateTerrainDialog::cancelButton() const
{
    return mCancel;
}

rt::Widget* CreateTerrainDialog::focusOnOpen()
{
    mName->selectAll();
    return mName;
}

void CreateTerrainDialog::onKeyPress(rt::KeyEvent& event)
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
