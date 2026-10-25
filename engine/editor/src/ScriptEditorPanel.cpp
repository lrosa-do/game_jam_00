#include "prisma3d/editor/ScriptEditorPanel.h"

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/editor/WidgetHost.h"

#include "platform.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <stdlib.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

ct::String leafOf(const ct::String& path)
{
    const std::size_t slash = path.rfind('/');
    if (slash == ct::String::npos || slash + 1 >= path.size()) return path;
    return path.substr(slash + 1);
}

} // namespace

ScriptEditorPanel::ScriptEditorPanel(EditorDocument& document)
        : mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mRoot(nullptr),
          mMenuBar(nullptr),
          mToolBar(nullptr),
          mTitle(nullptr),
          mEditor(nullptr),
          mOpenButton(-1),
          mSaveButton(-1),
          mCutButton(-1),
          mCopyButton(-1),
          mPasteButton(-1),
          mModified(false),
          mLoading(false),
          mControl(false)
{
}

ScriptEditorPanel::~ScriptEditorPanel()
{
    mLink->owner = nullptr;
}

bool ScriptEditorPanel::attach(rt::Widget* parent)
{
    if (mRoot || !parent) return false;
    const ct::Rc<Link> link = mLink;

    mRoot = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mRoot->setStretch(1.0f);
    mRoot->setPadding(0.0f);
    mRoot->setSpacing(0.0f);

    mMenuBar = mRoot->createChild<rt::MenuBar>();
    rt::Menu* file = mMenuBar->addMenu("File");
    file->addAction("Open...",
                [link]
                {
                    if (link->owner) link->owner->requestOpen();
                })
            ->setShortcut("Ctrl+O");
    file->addAction("Save",
                [link]
                {
                    if (link->owner) link->owner->save();
                })
            ->setShortcut("Ctrl+S");
    file->addAction("Save As...",
                [link]
                {
                    if (link->owner) link->owner->requestSaveAs();
                })
            ->setShortcut("Ctrl+Shift+S");
    rt::Menu* edit = mMenuBar->addMenu("Edit");
    edit->addAction("Undo",
                [link]
                {
                    if (link->owner) link->owner->undo();
                })
            ->setShortcut("Ctrl+Z");
    edit->addAction("Redo",
                [link]
                {
                    if (link->owner) link->owner->redo();
                })
            ->setShortcut("Ctrl+Y");
    edit->addSeparator();
    edit->addAction("Cut",
                [link]
                {
                    if (link->owner) link->owner->cut();
                })
            ->setShortcut("Ctrl+X");
    edit->addAction("Copy",
                [link]
                {
                    if (link->owner) link->owner->copy();
                })
            ->setShortcut("Ctrl+C");
    edit->addAction("Paste",
                [link]
                {
                    if (link->owner) link->owner->paste();
                })
            ->setShortcut("Ctrl+V");

    rt::Menu* view = mMenuBar->addMenu("View");
    view->addAction("Zoom In",
                [link]
                {
                    if (link->owner) link->owner->zoomIn();
                })
            ->setShortcut("Ctrl+=");
    view->addAction("Zoom Out",
                [link]
                {
                    if (link->owner) link->owner->zoomOut();
                })
            ->setShortcut("Ctrl+-");
    view->addAction("Reset Zoom",
                [link]
                {
                    if (link->owner) link->owner->zoomReset();
                })
            ->setShortcut("Ctrl+0");

    rt::BoxLayout* row = mRoot->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    row->setPadding(0.0f);
    row->setSpacing(8.0f);
    mToolBar = row->createChild<rt::ToolBar>();
    mToolBar->setDraggable(false);
    mToolBar->setButtonSize(EditorShell::kToolBarButtonSize);
    mToolBar->setIconPadding(EditorShell::kToolBarIconPadding);
    mOpenButton = mToolBar->addButton(EditorIcons::get("Load"), "Open (Ctrl+O)");
    mSaveButton = mToolBar->addButton(EditorIcons::get("Save"), "Save (Ctrl+S)");
    mToolBar->addSeparator();
    mCutButton = mToolBar->addButton(EditorIcons::get("ActionCut"), "Cut (Ctrl+X)");
    mCopyButton = mToolBar->addButton(EditorIcons::get("ActionCopy"), "Copy (Ctrl+C)");
    mPasteButton = mToolBar->addButton(EditorIcons::get("ActionPaste"), "Paste (Ctrl+V)");
    mToolBar->buttonClicked.connect(
            [link](int id)
            {
                if (link->owner) link->owner->toolClicked(id);
            });
    mTitle = row->createChild<rt::Label>(ct::String());
    mTitle->setLayoutAlign(rt::Align::Center);
    mTitle->setStretch(1.0f);

    mEditor = mRoot->createChild<rt::CodeEditor>();
    mEditor->setStretch(1.0f);
    mEditor->setShowLineNumbers(true);
    mEditor->setTabSize(4);
    mEditor->setShowIndentGuides(false);
    mEditor->setShowWhitespace(true);
    mEditor->setReadOnly(true);
    mEditor->textChanged.connect(
            [link]
            {
                if (!link->owner || link->owner->mLoading || link->owner->mModified) return;
                link->owner->mModified = true;
                link->owner->refreshTitle();
            });
    refreshTitle();
    return true;
}

bool ScriptEditorPanel::attached() const
{
    return mRoot != nullptr;
}

void ScriptEditorPanel::setRevealHandler(ct::Function<void()> handler)
{
    mReveal = static_cast<ct::Function<void()>&&>(handler);
}

void ScriptEditorPanel::open(const ct::String& path)
{
    if (!mEditor || path.empty()) return;
    if (path == mPath)
    {
        if (mReveal) mReveal();
        return;
    }
    if (!mModified)
    {
        load(path);
        return;
    }
    rt::MessageBox* box = rt::WidgetApp::instance().addFloat<rt::MessageBox>("Unsaved changes",
            ct::String("Save the changes to ") + leafOf(mPath) + "?", rt::MessageBox::YesNoCancel,
            rt::MessageBox::Question);
    const ct::Rc<Link> link = mLink;
    box->result.connect(
            [link, path](rt::MessageBox::Result result)
            {
                ScriptEditorPanel* owner = link->owner;
                if (!owner || result == rt::MessageBox::ResultCancel) return;
                if (result == rt::MessageBox::ResultYes && !owner->save()) return;
                owner->load(path);
            });
}

bool ScriptEditorPanel::load(const ct::String& path)
{
    char* text = file_read_text(path.c_str());
    if (!text)
    {
        log_error("editor: could not read the script %s", path.c_str());
        return false;
    }
    mLoading = true;
    mEditor->setHighlighterForFile(path);
    mEditor->setText(ct::String(text));
    mEditor->clearUndoHistory();
    mEditor->setReadOnly(false);
    mLoading = false;
    free(text);
    mPath = path;
    mModified = false;
    refreshTitle();
    if (mReveal) mReveal();
    return true;
}

bool ScriptEditorPanel::writeTo(const ct::String& path)
{
    if (!file_write_text(path.c_str(), mEditor->text().c_str()))
    {
        log_error("editor: could not write the script %s", path.c_str());
        return false;
    }
    if (path != mPath)
    {
        mPath = path;
        mEditor->setHighlighterForFile(path);
    }
    mModified = false;
    refreshTitle();
    return true;
}

bool ScriptEditorPanel::save()
{
    if (!mEditor || mEditor->readOnly()) return false;
    if (mPath.empty())
    {
        requestSaveAs();
        return false;
    }
    return writeTo(mPath);
}

void ScriptEditorPanel::requestOpen()
{
    if (!mEditor) return;
    rt::FileDialog* dialog = rt::WidgetApp::instance().addFloat<rt::FileDialog>("Open script");
    dialog->setMode(rt::FileDialog::Mode::Open);
    dialog->setFilter(kScriptFilter);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, EditorSettings::kFolderScript,
            &mDocument.project());
    const ct::Rc<Link> link = mLink;
    dialog->accepted.connect(
            [link, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderScript, path);
                if (link->owner) link->owner->open(path);
            });
}

void ScriptEditorPanel::requestSaveAs()
{
    if (!mEditor || mEditor->readOnly()) return;
    rt::FileDialog* dialog = rt::WidgetApp::instance().addFloat<rt::FileDialog>("Save script as");
    dialog->setMode(rt::FileDialog::Mode::Save);
    dialog->setFilter(kScriptFilter);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, EditorSettings::kFolderScript,
            &mDocument.project());
    if (!mPath.empty()) dialog->setFileName(leafOf(mPath));
    const ct::Rc<Link> link = mLink;
    dialog->accepted.connect(
            [link, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderScript, path);
                if (link->owner) link->owner->writeTo(path);
            });
}

void ScriptEditorPanel::cut()
{
    if (mEditor) mEditor->cut();
}

void ScriptEditorPanel::copy()
{
    if (mEditor) mEditor->copy();
}

void ScriptEditorPanel::paste()
{
    if (mEditor) mEditor->paste();
}

void ScriptEditorPanel::undo()
{
    if (mEditor) mEditor->undo();
}

void ScriptEditorPanel::redo()
{
    if (mEditor) mEditor->redo();
}

void ScriptEditorPanel::zoomIn()
{
    if (mEditor) mEditor->zoomIn();
}

void ScriptEditorPanel::zoomOut()
{
    if (mEditor) mEditor->zoomOut();
}

void ScriptEditorPanel::zoomReset()
{
    if (mEditor) mEditor->zoomReset();
}

void ScriptEditorPanel::update()
{
    if (!mToolBar) return;
    const bool editable = !mEditor->readOnly();
    const bool selection = mEditor->hasSelection();
    mToolBar->setItemEnabled(mSaveButton, editable);
    mToolBar->setItemEnabled(mCutButton, editable && selection);
    mToolBar->setItemEnabled(mCopyButton, selection);
    mToolBar->setItemEnabled(mPasteButton, editable);
}

bool ScriptEditorPanel::visibleInTree() const
{
    if (!mEditor) return false;
    for (const rt::Widget* widget = mEditor; widget; widget = widget->parent())
        if (!widget->isVisible()) return false;
    return true;
}

bool ScriptEditorPanel::focused() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return mEditor && app && app->focused() == mEditor;
}

bool ScriptEditorPanel::handleEvent(const InputEvent& event)
{
    if (event.type == InputEventType::MouseWheel)
    {
        if (!mControl || !visibleInTree() || WidgetHost::modalOpen() ||
                !mEditor->absoluteRect().contains(static_cast<float>(event.mouseWheel.x),
                        static_cast<float>(event.mouseWheel.y)))
            return false;
        if (event.mouseWheel.deltaY > 0.0f) zoomIn();
        else if (event.mouseWheel.deltaY < 0.0f)
            zoomOut();
        return true;
    }
    if (event.type != InputEventType::Key) return false;
    if (event.key.key == Key::LeftControl || event.key.key == Key::RightControl)
        mControl = event.key.pressed;
    if (!event.key.pressed) return false;
    if (!(event.key.modifiers & kModControl) || WidgetHost::modalOpen() || !focused()) return false;
    switch (event.key.key)
    {
        case Key::Equal:
        case Key::KpAdd:
            zoomIn();
            return true;
        case Key::Minus:
        case Key::KpSubtract:
            zoomOut();
            return true;
        case Key::Zero:
        case Key::Kp0:
            zoomReset();
            return true;
        default:
            break;
    }
    if (event.key.echo) return false;
    const bool shift = (event.key.modifiers & kModShift) != 0;
    switch (event.key.key)
    {
        case Key::S:
            if (shift) requestSaveAs();
            else
                save();
            return true;
        case Key::O:
            requestOpen();
            return true;
        default:
            return false;
    }
}

void ScriptEditorPanel::toolClicked(int id)
{
    if (id == mOpenButton) requestOpen();
    else if (id == mSaveButton)
        save();
    else if (id == mCutButton)
        cut();
    else if (id == mCopyButton)
        copy();
    else if (id == mPasteButton)
        paste();
}

void ScriptEditorPanel::refreshTitle()
{
    if (!mTitle) return;
    if (mPath.empty())
    {
        mTitle->setText("No script open");
        mTitle->setColor(rt::Theme::instance().textDisabled);
        return;
    }
    mTitle->setText(mModified ? leafOf(mPath) + " *" : leafOf(mPath));
    mTitle->setColor(rt::Theme::instance().textColor);
}

const ct::String& ScriptEditorPanel::path() const
{
    return mPath;
}

bool ScriptEditorPanel::modified() const
{
    return mModified;
}

rt::CodeEditor* ScriptEditorPanel::editor() const
{
    return mEditor;
}

} // namespace prisma3d
